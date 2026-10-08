#!/usr/bin/env python3
"""
Cut the Jetwing into pieces a Bambu Lab A1 can print, and bore the spar.

    python3 blender/print_parts.py
    blender --background --python blender/print_parts.py

BUILD VOLUME
    256 x 256 x 256 mm. The root chord is 260 mm, so the root section does
    not fit square on the bed at all -- it goes on diagonally, where 256 mm
    of bed gives 362 mm. The slicer will do that for you; the check below
    allows for it.

HOW THE PARTS SIT
    Wing sections print STANDING ON END, span axis vertical. That puts the
    spar bore straight up the Z axis, so it needs no support and comes out
    round, and it lays the layer lines across the chord where the skin wants
    them. It also means the limit on a section is 256 mm of SPAN, not of
    chord.

THE SPAR
    Sized in tools/spar.py from the real span loading: 19.8 N.m of root
    bending at 9 g ultimate. Two telescoping tubes, because an 8 mm tube
    slides inside a 10 x 8:

      A   10 x 8 mm, 900 mm long, centred      root spar AND wing joiner
      B    8 x 6 mm, 500 mm, each side         inserts 70 mm into A

    plus a 4 mm anti-rotation rod across the centre, because one round tube
    lets the panels rotate about it however well it fits.
"""

import math
import os
import sys

import bpy
import bmesh
from mathutils import Vector, Matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "print")

BED = 256.0
DIAG = BED * math.sqrt(2.0) - 12.0      # usable diagonal, minus a margin

SPAR = {
    "frac": 0.30,              # chordwise position, where the section is thickest
    "A_od": 10.0, "A_to": 455.0,
    "B_od": 8.0,  "B_from": 380.0, "B_to": 890.0,
    "fit": 0.40,               # added to diameter: a sliding fit after shrink
    "anti_od": 4.0, "anti_frac": 0.62, "anti_to": 190.0,
    "dowel": 3.0, "dowel_depth": 14.0,
}

# spanwise cuts, as fractions of the semi-span. Five per side keeps every
# section inside 256 mm of span with room for the joint faces.
CUTS = [0.0, 0.200, 0.400, 0.600, 0.800, 1.0]


def spar_point(y, frac):
    """The spar axis where it crosses a station, in the wing's own frame --
    taken through the same place() the skin uses, so the bore cannot drift
    away from the section it is supposed to sit inside."""
    c = JW.chord_at(y)
    z = JW.camber(frac, JW.P["camber"], JW.P["camber_pos"],
                  JW.P["reflex"], JW.P["reflex_start"])[0]
    p = JW.place([(frac, z)], c, JW.le_at(y), y,
                 y * math.tan(math.radians(JW.P["dihedral"])), JW.twist_at(y))[0]
    return Vector(p)


def cylinder(a, b, dia, name, coll, npts=28):
    """A capped cylinder between two points, built directly -- no primitives,
    so it lands exactly on the axis we computed."""
    a, b = Vector(a), Vector(b)
    ax = (b - a)
    L = ax.length
    ax = ax / L
    u = Vector((1.0, 0.0, 0.0))
    if abs(ax.dot(u)) > 0.9:
        u = Vector((0.0, 1.0, 0.0))
    v = ax.cross(u).normalized()
    u = v.cross(ax).normalized()
    r = dia / 2.0
    rings = []
    for end in (a, b):
        rings.append([end + (u * math.cos(t) + v * math.sin(t)) * r
                      for t in [2 * math.pi * i / npts for i in range(npts)]])
    verts, faces = [], []
    for ring in rings:
        verts += [tuple(p) for p in ring]
    for i in range(npts):
        j = (i + 1) % npts
        faces.append([i, j, npts + j, npts + i])
    faces.append(list(range(npts - 1, -1, -1)))
    faces.append([npts + i for i in range(npts)])
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in verts], [], faces)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def box(lo, hi, name, coll):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
         [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in v], [], f)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def boolean(target, cutter, op='DIFFERENCE'):
    m = target.modifiers.new("bool", 'BOOLEAN')
    m.operation = op
    m.object = cutter
    m.solver = 'EXACT'
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=m.name)


def watertight(ob):
    """A slicer needs a closed surface. Count the edges that are not shared by
    exactly two faces -- holes, cracks and internal walls all show up here,
    and none of them slice into anything sensible."""
    bm = bmesh.new()
    deps = bpy.context.evaluated_depsgraph_get()
    bm.from_mesh(ob.evaluated_get(deps).to_mesh())
    bad = sum(1 for e in bm.edges if not e.is_manifold)
    loose = sum(1 for v in bm.verts if not v.link_faces)
    n = len(bm.faces)
    bm.free()
    return bad, loose, n


SHELL = {"Fuselage": 1.2, "Duct": 1.0, "Inlet": 1.0}


def solidify(ob, thickness):
    """Give an open surface a real wall.

    The fuselage, duct and inlets were modelled as single surfaces -- correct
    for a render, unprintable, because a zero-thickness skin has nothing to
    slice. Solidify turns each into a closed shell of a stated thickness,
    which is also what you want on the plate: a controlled wall rather than
    a solid block the slicer has to hollow out again.
    """
    m = ob.modifiers.new("solid", 'SOLIDIFY')
    m.thickness = thickness * JW.MM
    m.offset = 1.0                 # grow inwards, keep the outer surface true
    m.use_even_offset = True
    m.use_rim = True
    m.use_rim_only = False
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=m.name)
    return ob


def dims_mm(ob):
    deps = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(deps)
    pts = [ev.matrix_world @ Vector(c) for c in ev.bound_box]
    return [(max(p[i] for p in pts) - min(p[i] for p in pts)) * 1000.0
            for i in range(3)]


def mirror_y(ob, name, coll):
    """A left-hand copy. Building both sides from the loft got them subtly
    different -- the dihedral term is written in signed y, so the left wing
    drooped instead of rising. One side, mirrored, cannot disagree with
    itself."""
    dup = ob.copy()
    dup.data = ob.data.copy()
    dup.name = name
    coll.objects.link(dup)
    dup.data.transform(Matrix.Diagonal((1.0, -1.0, 1.0, 1.0)))
    dup.data.flip_normals()
    return dup


def halve(ob, work, coll):
    """Cut a part in two across its longest axis, when it will not fit."""
    lo, hi = bounds_of(ob)
    ax = max(range(3), key=lambda i: hi[i] - lo[i])
    mid = 0.5 * (lo[ax] + hi[ax])
    out = []
    for k, (a, b) in enumerate(((lo[ax], mid), (mid, hi[ax]))):
        dup = ob.copy()
        dup.data = ob.data.copy()
        dup.name = f"{ob.name}{chr(97 + k)}"
        coll.objects.link(dup)
        blo = [lo[i] - 60 for i in range(3)]
        bhi = [hi[i] + 60 for i in range(3)]
        blo[ax], bhi[ax] = a, b
        cutter = box(blo, bhi, "cut", work)
        boolean(dup, cutter, 'INTERSECT')
        bpy.data.objects.remove(cutter, do_unlink=True)
        if len(dup.data.vertices):
            out.append(dup)
        else:
            bpy.data.objects.remove(dup, do_unlink=True)
    if out:
        bpy.data.objects.remove(ob, do_unlink=True)
    return out or [ob]


def fits(d):
    """Does it go on the bed? Tall things can be laid diagonally, so the two
    smaller dimensions only have to make the diagonal, not the side."""
    s = sorted(d)
    return s[2] <= BED and math.hypot(s[0], s[1]) <= DIAG and s[1] <= BED


# ==========================================================================
# the work
# ==========================================================================

def bore_wing(wing, work):
    """Stepped spar bore, plus the anti-rotation rod across the centre."""
    f = SPAR["frac"]
    cuts = []
    a0, a1 = spar_point(-SPAR["A_to"], f), spar_point(SPAR["A_to"], f)
    cuts.append(cylinder(a0, a1, SPAR["A_od"] + SPAR["fit"], "sparA", work))
    for sgn in (1.0, -1.0):
        b0 = spar_point(sgn * SPAR["B_from"], f)
        b1 = spar_point(sgn * SPAR["B_to"], f)
        cuts.append(cylinder(b0, b1, SPAR["B_od"] + SPAR["fit"],
                             f"sparB{sgn:+.0f}", work))
    g = SPAR["anti_frac"]
    cuts.append(cylinder(spar_point(-SPAR["anti_to"], g),
                         spar_point(SPAR["anti_to"], g),
                         SPAR["anti_od"] + SPAR["fit"], "anti", work))
    for c in cuts:
        boolean(wing, c)
        bpy.data.objects.remove(c, do_unlink=True)
    return wing


def dowels(part, y, work, n=2):
    """Two blind holes either side of the spar at a joint face, so a section
    cannot rotate about the spar while the glue goes off."""
    made = []
    for k, frac in enumerate((0.14, 0.52)):
        p = spar_point(y, frac)
        ax = (spar_point(y + 10.0, frac) - spar_point(y - 10.0, frac)).normalized()
        a = p - ax * (SPAR["dowel_depth"] / 2.0)
        b = p + ax * (SPAR["dowel_depth"] / 2.0)
        c = cylinder(a, b, SPAR["dowel"] + SPAR["fit"], f"dow{k}", work)
        made.append(c)
    for c in made:
        boolean(part, c)
        bpy.data.objects.remove(c, do_unlink=True)


def hinge_for(y, half):
    """Is this station inside a control-surface cut-out, and if so where does
    the fixed structure stop?"""
    f = abs(y) / half
    g = JW.P["gap"] / half
    for (a, b) in ((JW.P["flap_y0"], JW.P["flap_y1"]),
                   (JW.P["ail_y0"], JW.P["ail_y1"])):
        if a - g <= f <= b + g:
            return 1.0 - JW.P["flap_chord"]
    return 1.0


def section_cut(hinge, n_surf=46, n_close=7):
    """Section loop running only as far aft as `hinge`, closed by a straight
    face there, with a FIXED vertex count whatever the hinge is.

    The obvious way -- take the full section and clamp x to the hinge -- piles
    a dozen points onto one line and leaves sliver faces behind. They look
    harmless and they render fine, but an exact boolean lands on them and
    produces garbage: that is what mangled the spar bores first time round.
    Resampling to the hinge and closing it deliberately avoids making them.
    """
    P = JW.P
    m, pp, q = P["camber"], P["camber_pos"], P["reflex_start"]
    t, refl = P["thickness"], P["reflex"]

    def surf(x):
        yt = JW.thickness(x, t)
        yc, dy = JW.camber(x, m, pp, refl, q)
        th = math.atan(dy)
        return ((x - yt * math.sin(th), yc + yt * math.cos(th)),
                (x + yt * math.sin(th), yc - yt * math.cos(th)))

    up, lo = [], []
    for i in range(n_surf):
        # cosine spacing, dense at the leading edge where curvature is
        f = 0.5 * (1.0 - math.cos(math.pi * i / (n_surf - 1)))
        x = hinge * f
        u, l = surf(x)
        up.append(u)
        lo.append(l)
    u_h, l_h = up[-1], lo[-1]
    close = [(u_h[0] + (l_h[0] - u_h[0]) * (k + 1) / (n_close + 1),
              u_h[1] + (l_h[1] - u_h[1]) * (k + 1) / (n_close + 1))
             for k in range(n_close)]
    return up[::-1] + lo[1:] + close


def wing_segment(y0, y1, name, coll, n=14):
    """Build a print section straight from the parametric loft.

    NOT by cutting the wing. The wing object is several lofted runs merged
    into one mesh, so where two runs abut there are coincident internal
    faces: non-manifold, and an exact boolean on it produces nonsense --
    which is exactly what cutting it gave. Generating the section from the
    same sections the skin is made of is manifold by construction, lands on
    the surface to the micron, and cannot fail.

    Stations inside a flap cut-out have their aft points CLAMPED to the hinge
    line rather than dropped, so every section keeps the same vertex count
    and the loft stays well formed.
    """
    half = JW.P["span"] / 2.0
    dih = math.tan(math.radians(JW.P["dihedral"]))
    secs = []
    for i in range(n + 1):
        y = y0 + (y1 - y0) * i / n
        cut = section_cut(hinge_for(y, half))
        secs.append(JW.place(cut, JW.chord_at(y), JW.le_at(y), y,
                             abs(y) * dih, JW.twist_at(y)))
    v, f = JW.loft(secs, cap_start=True, cap_end=True)
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in v], [], f)
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    return ob


def control_segment(y0, y1, name, coll, n=10):
    """The matching piece of a flap or aileron: the part aft of the hinge."""
    hinge = 1.0 - JW.P["flap_chord"]
    dih = math.tan(math.radians(JW.P["dihedral"]))
    secs = []
    for i in range(n + 1):
        y = y0 + (y1 - y0) * i / n
        full = section_cut(1.0)
        cut = [(max(x, hinge), z) for (x, z) in full]
        secs.append(JW.place(cut, JW.chord_at(y), JW.le_at(y), y,
                             abs(y) * dih, JW.twist_at(y)))
    v, f = JW.loft(secs, cap_start=True, cap_end=True)
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in v], [], f)
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def split_span(src, name, cuts, work, parts_coll, half):
    """Cut one object into spanwise sections, both wings."""
    out = []
    lo, hi = bounds_of(src)
    for sgn in (1.0, -1.0):
        for i in range(len(cuts) - 1):
            y0, y1 = cuts[i] * half, cuts[i + 1] * half
            if sgn < 0:
                y0, y1 = -y1, -y0
            dup = src.copy()
            dup.data = src.data.copy()
            parts_coll.objects.link(dup)
            dup.name = f"{name}_{'R' if sgn > 0 else 'L'}{i + 1}"
            cutter = box((lo[0] - 50, y0, lo[2] - 50), (hi[0] + 50, y1, hi[2] + 50),
                         "cut", work)
            boolean(dup, cutter, 'INTERSECT')
            bpy.data.objects.remove(cutter, do_unlink=True)
            if len(dup.data.vertices) == 0:
                bpy.data.objects.remove(dup, do_unlink=True)
                continue
            for y in (y0, y1):
                if abs(abs(y) - half) > 1.0 and abs(y) > 1.0:
                    dowels(dup, y, work)
            out.append(dup)
    return out


def bounds_of(ob):
    deps = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(deps)
    pts = [(ev.matrix_world @ Vector(c)) * 1000.0 for c in ev.bound_box]
    return ([min(p[i] for p in pts) for i in range(3)],
            [max(p[i] for p in pts) for i in range(3)])


def split_x(src, name, n, work, parts_coll):
    """Cut a long body into sections along its length."""
    lo, hi = bounds_of(src)
    out = []
    step = (hi[0] - lo[0]) / n
    for i in range(n):
        dup = src.copy()
        dup.data = src.data.copy()
        parts_coll.objects.link(dup)
        dup.name = f"{name}_{i + 1}"
        cutter = box((lo[0] + i * step, lo[1] - 50, lo[2] - 50),
                     (lo[0] + (i + 1) * step, hi[1] + 50, hi[2] + 50),
                     "cut", work)
        boolean(dup, cutter, 'INTERSECT')
        bpy.data.objects.remove(cutter, do_unlink=True)
        if len(dup.data.vertices) == 0:
            bpy.data.objects.remove(dup, do_unlink=True)
            continue
        out.append(dup)
    return out


def apply_mirror(ob):
    """Bake the Mirror modifier, so a part is a real two-sided object before
    it gets cut up."""
    for m in list(ob.modifiers):
        if m.type == 'MIRROR':
            bpy.context.view_layer.objects.active = ob
            bpy.ops.object.modifier_apply(modifier=m.name)


def export(parts):
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for ob in parts:
        for o in bpy.context.scene.objects:
            o.select_set(False)
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        path = os.path.join(OUT, ob.name + ".stl")
        try:
            bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True,
                                  global_scale=1000.0)
        except AttributeError:
            bpy.ops.export_mesh.stl(filepath=path, use_selection=True,
                                    global_scale=1000.0)
        d = dims_mm(ob)
        nm, lo, nf = watertight(ob)
        rows.append((ob.name, d, fits(d), nm, lo, nf))
    return rows


def main():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    JW.build()
    half = JW.P["span"] / 2.0

    work = bpy.data.collections.new("work")
    bpy.context.scene.collection.children.link(work)
    parts_coll = bpy.data.collections.new("parts")
    bpy.context.scene.collection.children.link(parts_coll)

    src = bpy.data.collections[JW.COLL]
    for ob in src.objects:
        apply_mirror(ob)
    for nm, t in SHELL.items():
        ob = bpy.data.objects.get(nm)
        if ob:
            solidify(ob, t)

    parts = []

    # --- wing sections, generated rather than cut ---
    f = SPAR["frac"]
    right = []
    for i in range(len(CUTS) - 1):
        y0, y1 = CUTS[i] * half, CUTS[i + 1] * half
        ob = wing_segment(y0, y1, f"wing_R{i + 1}", parts_coll)
        for (od, lo_y, hi_y) in ((SPAR["A_od"], 0.0, SPAR["A_to"]),
                                 (SPAR["B_od"], SPAR["B_from"], SPAR["B_to"])):
            s0, s1 = max(y0, lo_y), min(y1, hi_y)
            if s1 - s0 < 2.0:
                continue
            c = cylinder(spar_point(s0 - 14.0, f), spar_point(s1 + 14.0, f),
                         od + SPAR["fit"], "bore", work)
            boolean(ob, c)
            bpy.data.objects.remove(c, do_unlink=True)
        if y0 < 1.0:
            c = cylinder(spar_point(-14.0, SPAR["anti_frac"]),
                         spar_point(SPAR["anti_to"], SPAR["anti_frac"]),
                         SPAR["anti_od"] + SPAR["fit"], "anti", work)
            boolean(ob, c)
            bpy.data.objects.remove(c, do_unlink=True)
        right.append(ob)

    # --- control surfaces, in printable lengths ---
    for (nm, a, b, n) in (("flap", JW.P["flap_y0"], JW.P["flap_y1"], 2),
                          ("aileron", JW.P["ail_y0"], JW.P["ail_y1"], 3)):
        g = JW.P["gap"]
        y0, y1 = a * half + g, b * half - g
        for i in range(n):
            right.append(control_segment(y0 + (y1 - y0) * i / n,
                                         y0 + (y1 - y0) * (i + 1) / n,
                                         f"{nm}_R{i + 1}", parts_coll))

    parts = []
    for ob in right:
        parts.append(ob)
        parts.append(mirror_y(ob, ob.name.replace("_R", "_L"), parts_coll))

    fus = bpy.data.objects["Fuselage"]
    parts += split_x(fus, "fuselage", 4, work, parts_coll)
    for nm in ("Fin", "Rudder", "Inlet", "Duct", "Stator", "Motor"):
        ob = bpy.data.objects.get(nm)
        if ob:
            ob.name = "part_" + nm
            parts.append(ob)

    # anything still too big gets cut across its longest axis until it fits
    final = []
    for ob in parts:
        queue, guard = [ob], 0
        while queue and guard < 12:
            guard += 1
            p = queue.pop(0)
            if fits(dims_mm(p)) or len(p.data.vertices) == 0:
                final.append(p)
            else:
                queue.extend(halve(p, work, parts_coll))
        final.extend(queue)
    parts = final

    rows = export(parts)
    print(f"\n{'part':20s} {'X':>7s} {'Y':>7s} {'Z':>7s} {'faces':>7s}  "
          f"fits {BED:.0f}?  watertight?")
    bad = leaky = 0
    for (nm, d, ok, nman, loose, nf) in sorted(rows):
        wt = "yes" if (nman == 0 and loose == 0) else f"NO ({nman} open edges)"
        print(f"  {nm:18s} {d[0]:7.1f} {d[1]:7.1f} {d[2]:7.1f} {nf:7d}  "
              f"{'yes' if ok else 'NO':>7s}      {wt}")
        bad += 0 if ok else 1
        leaky += 0 if (nman == 0 and loose == 0) else 1
    print(f"\n  {len(rows)} parts, {bad} too big, {leaky} not watertight")
    print(f"  written to {OUT}/")
    return bad


if __name__ == "__main__":
    main()
