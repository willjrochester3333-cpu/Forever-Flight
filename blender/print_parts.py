#!/usr/bin/env python3
"""
Cut the Jetwing into pieces a Bambu Lab A2L can print, and bore the spar.

    python3 blender/print_parts.py
    blender --background --python blender/print_parts.py

BUILD VOLUME
    330 x 320 x 325 mm -- twice the volume of the A1 this was first cut for,
    and, in the dimension that actually decides the part count, 325 mm of
    height against 256.

    Two things fall out of that. The root chord is 260 mm, so a root section
    now stands SQUARE on the plate with 60 mm to spare: the A1 cut had to
    lay it on the diagonal and trust the slicer to find that placement. And
    a section may be 325 mm of span instead of 256, which takes the wing
    from five sections a side to four, the ailerons from three to two, the
    fuselage from four rings to two, and lets the winglet stay attached to
    the outboard section instead of being a separate glued-on part.

HOW THE PARTS SIT
    Wing sections print STANDING ON END, span axis vertical. That puts the
    spar bore straight up the Z axis, so it needs no support and comes out
    round, and it lays the layer lines across the chord where the skin wants
    them. It also means the limit on a section is 325 mm of SPAN, not of
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

BED = (330.0, 320.0, 325.0)     # X, Y, Z -- the A2L is not a cube
MARGIN = 6.0                    # brim and skirt want the edge of the plate

SPAR = {
    "frac": 0.30,              # chordwise position, where the section is thickest
    "A_od": 10.0, "A_to": 455.0,
    "B_od": 8.0,  "B_from": 380.0, "B_to": 890.0,
    "fit": 0.40,               # added to diameter: a sliding fit after shrink
    "anti_od": 4.0, "anti_frac": 0.62, "anti_to": 190.0,
    # Joint dowels: 4 mm rod, 22 mm long, 11 mm into each face. Up from
    # 3 x 14 -- the sections are half again as long now, so the joint carries
    # more bending, and a 4 mm rod in an 11 mm blind hole is still a hole a
    # printer makes honestly.
    "dowel": 4.0, "dowel_depth": 22.0, "dowel_fracs": (0.14, 0.60),
}

# 9 g servos (SG90 class). The body is 22.5 x 11.8 x 22.7 and it lies with
# its output shaft SPANWISE, so the arm sweeps in a chordwise-vertical plane
# and drives the surface directly. That costs only 11.8 mm of section depth
# instead of 22.7, which is the whole reason it fits.
#
# The bay sits at 45% chord -- just aft of the spar at 30%, and close enough
# to maximum thickness to have room. At the hinge line itself there is only
# 11-15 mm of section, so a servo buried under the control surface does NOT
# fit; it is driven by a pushrod instead.
SERVO = {
    "body": (22.5, 11.8, 22.7),     # chordwise, vertical, spanwise
    "clear": 1.3,                   # all round the body
    "chord_frac": 0.45,
    "stations": {"flap": 300.0, "aileron": 570.0},
    "pushrod_bore": 3.4,            # guide for a 2 mm rod in a 3 mm tube
    "horn_frac": 0.735,             # just ahead of the hinge at 0.74
    "wire_bore": 6.0,
    # 50% chord, not 52%. The bay's aft wall lands at 50.8% chord, so on 52%
    # the channel only grazed it -- a 0.4 mm sliver of overlap. On 50% it runs
    # decisively through the bay, which is what the servo lead has to do
    # anyway: the lead leaves the servo and joins the loom in one move.
    "wire_frac": 0.50,
    "wire_to": 620.0,               # outboard end of the loom run
    "hatch_t": 1.8,
    "hatch_lip": 3.5,
    "boss_h": 7.0, "boss_d": 5.4, "boss_hole": 1.9,
}

# Spanwise cuts, as fractions of the semi-span. FOUR per side, 257.5 mm of
# span each, inside the 325 mm height with 67 mm spare. Three a side would
# want 343 mm and does not go, so four is the fewest the A2L allows.
#
# Equal quarters put the joints at y = 257.5, 515 and 772.5 mm. That is 30 mm
# or more clear of both servo bays, and clear of the spar step at y = 455,
# which is what the spacing has to miss.
CUTS = [0.0, 0.25, 0.50, 0.75, 1.0]


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


def skin_point(y, xf, lower=True):
    """A point on the wing skin at a station and chord fraction."""
    P = JW.P
    m, pp, q = P["camber"], P["camber_pos"], P["reflex_start"]
    t, refl = P["thickness"], P["reflex"]
    yt = JW.thickness(xf, t)
    yc, dy = JW.camber(xf, m, pp, refl, q)
    th = math.atan(dy)
    pt = ((xf + yt * math.sin(th), yc - yt * math.cos(th)) if lower
          else (xf - yt * math.sin(th), yc + yt * math.cos(th)))
    dih = math.tan(math.radians(P["dihedral"]))
    return Vector(JW.place([pt], JW.chord_at(y), JW.le_at(y), y,
                           abs(y) * dih, JW.twist_at(y))[0])


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
    """Apply a boolean -- and check it actually did the cut.

    The exact solver has no failure return. Handed a target it cannot reason
    about it produces something plausible-looking, and the one thing nothing
    downstream catches is the cutter coming back in place of the part: a 6 mm
    tube is watertight, fits the bed and slices, so it passes every other
    check in this file and goes in the box. Two wing sections shipped that way.

    A DIFFERENCE can only remove material, so it cannot halve the face count
    or shrink the bounding box. If it did, the solver did not do a difference.
    """
    before = len(target.data.polygons)
    lo0, hi0 = bounds_of(target)
    m = target.modifiers.new("bool", 'BOOLEAN')
    m.operation = op
    m.object = cutter
    m.solver = 'EXACT'
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=m.name)
    if op == 'DIFFERENCE':
        after = len(target.data.polygons)
        lo1, hi1 = bounds_of(target)
        shrunk = max(max(lo1[i] - lo0[i], hi0[i] - hi1[i]) for i in range(3))
        if after < before // 2 or shrunk > 5.0:
            raise RuntimeError(
                "boolean with %s collapsed %s: %d -> %d faces, bounding box "
                "shrank %.1f mm. The solver returned something that is not "
                "the cut -- check the target for self-intersection."
                % (cutter.name, target.name, before, after, shrunk))


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
    """Does it go on the plate?

    The A1 was a cube, so one number answered this. The A2L is 330 x 320 x
    325, and a part has three dimensions to hand out among three different
    limits. Sorting both lists descending and matching largest to largest is
    the optimal assignment when every constraint is an upper bound, so this
    decides it exactly -- there is nothing to search.

    Deliberately no diagonal placement. The A1 cut NEEDED it, because a
    260 mm root chord does not go on a 256 mm bed any other way, and a part
    that only fits cornerwise is a part a slicer can quietly place wrong.
    Nothing here needs it now; if something ever does, it is a sign the cut
    is wrong rather than the check.
    """
    for want, lim in zip(sorted(d, reverse=True), sorted(BED, reverse=True)):
        if want > lim - MARGIN:
            return False
    return True


# ==========================================================================
# the work
# ==========================================================================

def servo_bay(ob, y, work, S=SERVO):
    """Pocket for one servo, opening through the LOWER skin.

    Cut as an axis-aligned box: the section's chord already lies along x, and
    the local twist and dihedral are both about 2 degrees, which over a 23 mm
    box is a couple of tenths of a millimetre. Not worth a rotated cutter.
    """
    L, T, H = S["body"]
    cl = S["clear"]
    lo_skin = skin_point(y, S["chord_frac"], lower=True)
    up_skin = skin_point(y, S["chord_frac"], lower=False)
    x0 = lo_skin.x - (L + 2 * cl) / 2.0
    x1 = lo_skin.x + (L + 2 * cl) / 2.0
    y0, y1 = y - (H + 2 * cl) / 2.0, y + (H + 2 * cl) / 2.0
    z0 = lo_skin.z - 12.0                      # well clear, below the skin
    z1 = lo_skin.z + T + 2 * cl
    if z1 > up_skin.z - 2.0:                   # never breach the upper skin
        z1 = up_skin.z - 2.0
    c = box((x0, y0, z0), (x1, y1, z1), "bay", work)
    boolean(ob, c)
    bpy.data.objects.remove(c, do_unlink=True)

    # pushrod guide, from the bay aft to just ahead of the hinge
    a = skin_point(y, S["chord_frac"] + 0.02, lower=True) + Vector((0, 0, 6.0))
    b = skin_point(y, S["horn_frac"], lower=True) + Vector((0, 0, -4.0))
    g = cylinder(a, b, S["pushrod_bore"], "rod", work)
    boolean(ob, g)
    bpy.data.objects.remove(g, do_unlink=True)
    return ob


def swept_bore(path, dia, name, coll, npts=16, extend=12.0):
    """A tube following a polyline: rings perpendicular to the local tangent.

    The ends are run on straight by `extend` so the cutter leaves the part
    through its faces rather than stopping flush with one, which would be
    coplanar and is the one thing an exact boolean reliably hates.
    """
    pts = [Vector(p) for p in path]
    if extend:
        pts.insert(0, pts[0] + (pts[0] - pts[1]).normalized() * extend)
        pts.append(pts[-1] + (pts[-1] - pts[-2]).normalized() * extend)
    r = dia / 2.0
    rings = []
    for i, p in enumerate(pts):
        ax = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        u = Vector((1.0, 0.0, 0.0))
        if abs(ax.dot(u)) > 0.9:
            u = Vector((0.0, 1.0, 0.0))
        v = ax.cross(u).normalized()
        u = v.cross(ax).normalized()
        rings.append([p + (u * math.cos(t) + v * math.sin(t)) * r
                      for t in [2 * math.pi * k / npts for k in range(npts)]])
    verts, faces = [], []
    for ring in rings:
        verts += [tuple(q) for q in ring]
    for sg in range(len(rings) - 1):
        a, b = sg * npts, (sg + 1) * npts
        for i in range(npts):
            j = (i + 1) % npts
            faces.append([a + i, a + j, b + j, b + i])
    faces.append(list(range(npts - 1, -1, -1)))
    o = (len(rings) - 1) * npts
    faces.append([o + i for i in range(npts)])
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(q) * JW.MM for q in verts], [], faces)
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def wire_channel(ob, y0, y1, work, S=SERVO):
    """Spanwise bore for the servo loom, so wiring threads through the joints
    instead of being fished through them.

    SWEPT along the mid-thickness line, not driven straight between its two
    ends. The wing is swept and tapered, so the 50% chord line is not parallel
    to anything: a single straight cutter across a 257 mm section sits on 50%
    chord at both joints and wanders 11.3 mm off it in the middle. Swept, the
    channel stays mid-section the whole way and meets the next section's
    channel exactly, instead of stepping at every joint.
    """
    n = max(2, int(round((y1 - y0) / 18.0)))
    path = [((skin_point(y, S["wire_frac"], lower=True)
              + skin_point(y, S["wire_frac"], lower=False)) / 2.0)
            for y in (y0 + (y1 - y0) * i / n for i in range(n + 1))]
    c = swept_bore(path, S["wire_bore"], "loom", work)
    boolean(ob, c)
    bpy.data.objects.remove(c, do_unlink=True)
    return ob


def servo_hatch(y, name, coll, S=SERVO):
    """The cover, as a separate printed part.

    The servo screws to THIS on the bench, then the whole assembly drops into
    the bay. Fishing a servo into a closed pocket through its own hole is the
    worst job on a build like this, and it makes the servo unserviceable
    afterwards.
    """
    L, T, H = S["body"]
    cl, lip, th = S["clear"], S["hatch_lip"], S["hatch_t"]
    xf0 = S["chord_frac"] - (L / 2.0 + cl + lip) / JW.chord_at(y)
    xf1 = S["chord_frac"] + (L / 2.0 + cl + lip) / JW.chord_at(y)
    yy0, yy1 = y - (H / 2.0 + cl + lip), y + (H / 2.0 + cl + lip)

    nx, ny = 9, 5
    outer, inner = [], []
    for i in range(ny):
        yv = yy0 + (yy1 - yy0) * i / (ny - 1)
        ro, ri = [], []
        for j in range(nx):
            xf = xf0 + (xf1 - xf0) * j / (nx - 1)
            p = skin_point(yv, xf, lower=True)
            ro.append(tuple(p))
            ri.append((p.x, p.y, p.z + th))
        outer.append(ro)
        inner.append(ri)

    verts, faces = [], []
    def grid(g):
        base = len(verts)
        for row in g:
            verts.extend(row)
        return base
    bo, bi = grid(outer), grid(inner)
    for i in range(ny - 1):
        for j in range(nx - 1):
            a0 = bo + i * nx + j
            faces.append([a0, a0 + 1, a0 + nx + 1, a0 + nx])
            b0 = bi + i * nx + j
            faces.append([b0, b0 + nx, b0 + nx + 1, b0 + 1])
    for j in range(nx - 1):                       # fore and aft edges
        faces.append([bo + j, bo + nx * (0) + j + 1,
                      bi + j + 1, bi + j])
        o = nx * (ny - 1)
        faces.append([bo + o + j + 1, bo + o + j, bi + o + j, bi + o + j + 1])
    for i in range(ny - 1):                       # side edges
        faces.append([bo + i * nx, bo + (i + 1) * nx,
                      bi + (i + 1) * nx, bi + i * nx])
        e = nx - 1
        faces.append([bo + (i + 1) * nx + e, bo + i * nx + e,
                      bi + i * nx + e, bi + (i + 1) * nx + e])

    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in verts], [], faces)
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


def dowels(part, y, work):
    """Blind holes either side of the spar at a joint face, so a section
    cannot rotate about the spar while the glue goes off.

    Both holes run PARALLEL TO THE SPAR, not along their own chord lines.
    The 14, 30 and 60 per cent chord lines each have their own sweep: over a
    22 mm dowel the 60 per cent line diverges from the spar by 0.49 mm, which
    is more than the fit clearance, so three bores each on their own axis
    would simply refuse to go together. Shared direction, offset origin.

    The aft hole used to sit at 52 per cent chord, which is exactly where the
    servo loom channel runs -- a 5 mm bore swallowed the 3.4 mm dowel hole
    whole at every joint inboard of y = 620 and located nothing at all.
    """
    f = SPAR["frac"]
    ax = (spar_point(y + 10.0, f) - spar_point(y - 10.0, f)).normalized()
    h = SPAR["dowel_depth"] / 2.0
    for k, frac in enumerate(SPAR["dowel_fracs"]):
        p = spar_point(y, frac)
        c = cylinder(p - ax * h, p + ax * h,
                     SPAR["dowel"] + SPAR["fit"], f"dow{k}", work)
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


def section_cut(hinge, n_surf=46, n_close=0, m=None, refl=None):
    """Section loop running only as far aft as `hinge`, closed by a straight
    face there, with a FIXED vertex count whatever the hinge is.

    The obvious way -- take the full section and clamp x to the hinge -- piles
    a dozen points onto one line and leaves sliver faces behind. They look
    harmless and they render fine, but an exact boolean lands on them and
    produces garbage: that is what mangled the spar bores first time round.
    Resampling to the hinge and closing it deliberately avoids making them.

    The closing strip carries NO interior points (n_close = 0): the two
    surfaces are joined by one edge and the loft turns that into the hinge
    face. Interior points along it are exactly collinear, and a cap n-gon
    with a collinear run triangulates into zero-area slivers -- which count
    as self-intersections and were the last ones left in the bundle, one to
    three per part, all of them on the end cap at 0.74c. The strip was only
    ever there to add mesh density to a face nothing looks at.
    """
    P = JW.P
    pp, q, t = P["camber_pos"], P["reflex_start"], P["thickness"]
    m = P["camber"] if m is None else m
    refl = P["reflex"] if refl is None else refl

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
    # From the LOWER point back up to the UPPER one, because that is where
    # the loop has got to: up[::-1] runs trailing-to-leading along the top,
    # lo[1:] runs leading-to-trailing along the bottom, and the closing strip
    # has to climb back to where it started.
    #
    # Interpolating the other way -- which is how this was written -- makes
    # the outline arrive at the lower surface, jump back up to the upper one,
    # walk down again and only then close. The closing strip retraces the
    # hinge line twice, so the lofted body INTERSECTS ITSELF: 338 self-
    # intersecting face pairs in a single wing section, every one of them at
    # 0.74c.
    #
    # Nothing shows that in a render, and the mesh still counts as watertight.
    # What it does is make Blender's exact boolean solver unpredictable -- it
    # does not report failure, it returns something plausible instead. Here it
    # returned the CUTTER in place of the part, so a wing section came out as
    # a 6 mm tube that was watertight, fitted the bed, and sliced. The A1 cut
    # carried the same bug and happened to get the right answer.
    close = [(l_h[0] + (u_h[0] - l_h[0]) * (k + 1) / (n_close + 1),
              l_h[1] + (u_h[1] - l_h[1]) * (k + 1) / (n_close + 1))
             for k in range(n_close)]
    return up[::-1] + lo[1:] + close


def winglet_sections(n=10):
    """The winglet loft, in the SAME section form the wing segments use.

    jetwing.py lofts the winglet on inside the Wing object, and the print
    sections are generated from the planform rather than cut out of that
    object -- so nothing ever emitted a winglet, and the plate held an
    aircraft with 150 mm winglets in the aero model and none in the box.

    It stays attached to the outboard section rather than becoming a part of
    its own. A 325 mm plate has room, and a glued butt joint at the tip is
    the worst place on the aircraft to put one: the winglet is a cantilever
    in side load and the tip section is 11 mm thick, so there is nothing
    there to pin into. Lofted on, there is no joint to fail.

    Same stations and the same wash-out of camber and reflex as jetwing.py,
    so the printed winglet is the modelled winglet and not a near miss.
    """
    P = JW.P
    half = P["span"] / 2.0
    cant = math.radians(P["winglet_cant"])
    wsw = math.tan(math.radians(P["winglet_sweep"]))
    rise = half * math.tan(math.radians(P["dihedral"]))
    out = []
    for i in range(1, n + 1):
        t = i / float(n)
        h = P["winglet_h"] * t
        c = JW.chord_at(half) * (1.0 - (1.0 - P["winglet_taper"]) * t)
        cut = section_cut(1.0, m=P["camber"] * (1.0 - t),
                          refl=P["reflex"] * (1.0 - t))
        out.append(JW.place(cut, c, JW.le_at(half) + h * wsw,
                            half + h * math.cos(cant),
                            rise + h * math.sin(cant), JW.twist_at(half)))
    return out


def wing_segment(y0, y1, name, coll, n=14, winglet=False):
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
    if winglet:
        # the t = 0 winglet section IS the tip section, so it is skipped and
        # the loft runs straight on through: one continuous surface, manifold
        # by construction, no joint
        secs.extend(winglet_sections())
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


def section_aft(front, n_surf=40, n_close=0):
    """The part of the section AFT of the hinge, resampled, with the hinge
    face closed deliberately.

    The old way took the full section and clamped x into [hinge, 1]. That
    piles every point ahead of the hinge onto the hinge line while KEEPING
    its own thickness, so the flap came out carrying a zero-width flap of
    surface standing proud of its own hinge face, reaching up to maximum
    thickness. 1,477 self-intersecting face pairs in one aileron, and the
    normals inside-out with it, in parts that go straight to a slicer.
    """
    P = JW.P
    pp, q, t = P["camber_pos"], P["reflex_start"], P["thickness"]
    m, refl = P["camber"], P["reflex"]

    def surf(x):
        yt = JW.thickness(x, t)
        yc, dy = JW.camber(x, m, pp, refl, q)
        th = math.atan(dy)
        return ((x - yt * math.sin(th), yc + yt * math.cos(th)),
                (x + yt * math.sin(th), yc - yt * math.cos(th)))

    up, lo = [], []
    for i in range(n_surf):
        f = 0.5 * (1.0 - math.cos(math.pi * i / (n_surf - 1)))
        x = front + (1.0 - front) * f
        u, l = surf(x)
        up.append(u)
        lo.append(l)
    u0, l0 = up[0], lo[0]
    close = [(u0[0] + (l0[0] - u0[0]) * (k + 1) / (n_close + 1),
              u0[1] + (l0[1] - u0[1]) * (k + 1) / (n_close + 1))
             for k in range(n_close)]
    # trailing edge forward along the top, DOWN the hinge face, then back
    # along the bottom; the blunt trailing edge closes the loop itself
    return up[::-1] + close + lo


def control_segment(y0, y1, name, coll, n=10):
    """The matching piece of a flap or aileron: the part aft of the hinge."""
    hinge = 1.0 - JW.P["flap_chord"]
    dih = math.tan(math.radians(JW.P["dihedral"]))
    secs = []
    for i in range(n + 1):
        y = y0 + (y1 - y0) * i / n
        secs.append(JW.place(section_aft(hinge), JW.chord_at(y), JW.le_at(y),
                             y, abs(y) * dih, JW.twist_at(y)))
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
    """Write the STLs, clearing the directory first.

    Without the clear, a renamed or re-split part leaves its old file behind
    and the folder quietly accumulates stale geometry -- three dead parts
    from earlier splits were sitting in there, and a bundle built from the
    directory shipped 40 files for a 37-part aircraft.
    """
    if os.path.isdir(OUT):
        for f in os.listdir(OUT):
            if f.endswith(".stl"):
                os.remove(os.path.join(OUT, f))
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
    last = len(CUTS) - 2
    for i in range(len(CUTS) - 1):
        y0, y1 = CUTS[i] * half, CUTS[i + 1] * half
        ob = wing_segment(y0, y1, f"wing_R{i + 1}", parts_coll,
                          winglet=(i == last))
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

        # Loom channel BEFORE the bay. Order matters: a big box is a robust
        # cutter against a lofted skin, whereas a long thin cylinder meeting
        # a face the box has just made is the fragile way round.
        w0, w1 = max(y0, 0.0), min(y1, SERVO["wire_to"])
        if w1 - w0 > 2.0:
            wire_channel(ob, w0, w1, work)

        # servo bay, where one falls inside this section
        for (nm, ys) in SERVO["stations"].items():
            if y0 <= ys < y1:
                servo_bay(ob, ys, work)
                right.append(servo_hatch(ys, f"hatch_{nm}_R", parts_coll))

        # dowel holes, on the joint faces only -- the centreline face is
        # located by the spar itself and the tip face no longer exists
        for y in (y0, y1):
            if y > 1.0 and abs(y - half) > 1.0:
                dowels(ob, y, work)

        right.append(ob)

    # --- control surfaces, in printable lengths ---
    # flap band is 365 mm and aileron 406 mm, so both go in two pieces on a
    # 325 mm plate. On the A1 the aileron needed three.
    for (nm, a, b, n) in (("flap", JW.P["flap_y0"], JW.P["flap_y1"], 2),
                          ("aileron", JW.P["ail_y0"], JW.P["ail_y1"], 2)):
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
    # THREE rings of 232 mm, against four on the A1. Two would be 348 mm and
    # the plate is 330: it was tried, and the generic "halve anything that
    # does not fit" fallback cut them back to four anyway -- and left one of
    # them non-manifold, because a mid-body cut through a 1.2 mm shell is not
    # something to do by accident. The pod is 696 mm over the tailcone, not
    # the 694.6 mm measured in blender/measure.py, not the 620 mm of
    # pod_len.
    #
    # The cross-section is a 96 x 104 rounded superellipse rather than a
    # circle, so each butt joint keys itself: there is one way the rings go
    # together and it is obvious by feel.
    parts += split_x(fus, "fuselage", 3, work, parts_coll)
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
          f"fits A2L?  watertight?")
    bad = leaky = 0
    for (nm, d, ok, nman, loose, nf) in sorted(rows):
        wt = "yes" if (nman == 0 and loose == 0) else f"NO ({nman} open edges)"
        print(f"  {nm:18s} {d[0]:7.1f} {d[1]:7.1f} {d[2]:7.1f} {nf:7d}  "
              f"{'yes' if ok else 'NO':>7s}      {wt}")
        bad += 0 if ok else 1
        leaky += 0 if (nman == 0 and loose == 0) else 1
    print(f"\n  {len(rows)} parts, {bad} too big for "
          f"{BED[0]:.0f}x{BED[1]:.0f}x{BED[2]:.0f}, {leaky} not watertight")
    print(f"  written to {OUT}/")
    return bad


if __name__ == "__main__":
    main()
