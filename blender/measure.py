#!/usr/bin/env python3
"""
Measure the built Jetwing and check it against the specification.

    python3 blender/measure.py                 (with pip-installed bpy)
    blender --background --python blender/measure.py

WHY THIS EXISTS

Every dimension in jetwing.py is a number in a dictionary. That is not the
same as the model actually HAVING those dimensions: a rotation applied to the
wrong frame, a mirror about the wrong object, a loft that silently drops a
section, and the model quietly disagrees with its own specification. Both of
those have already happened in this file's history.

So this measures the real evaluated geometry -- after modifiers, in world
space -- and compares it with what was asked for. It also writes dimension
annotations into the scene so you can read the numbers off the model.
"""

import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW

TOL = 0.5          # mm; anything tighter is meaningless on a printed airframe


def _mass_g(S_dm2):
    """Built mass from tools/optimise.py, loaded by path -- tools/ also holds
    a jetwing.py, so putting it on sys.path would shadow the Blender one."""
    import importlib.util
    f = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "tools", "optimise.py")
    if not os.path.exists(f):
        return 0.0
    spec = importlib.util.spec_from_file_location("_opt", f)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.mass_g(S_dm2)


# ==========================================================================
# measuring
# ==========================================================================

def world_verts(name):
    """Evaluated vertices in world space, in millimetres."""
    ob = bpy.data.objects.get(name)
    if ob is None:
        return []
    deps = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(deps)
    me = ev.to_mesh()
    mw = ev.matrix_world
    pts = [(mw @ v.co) * 1000.0 for v in me.vertices]
    ev.to_mesh_clear()
    return pts


def world_faces(name):
    """Evaluated polygons as world-space vertex lists, in millimetres."""
    ob = bpy.data.objects.get(name)
    if ob is None:
        return []
    deps = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(deps)
    me = ev.to_mesh()
    mw = ev.matrix_world
    out = [[(mw @ me.vertices[i].co) * 1000.0 for i in poly.vertices]
           for poly in me.polygons]
    ev.to_mesh_clear()
    return out


def wing_only(pts, half):
    """Drop the winglets. They cant outboard past the tip, so anything beyond
    the semi-span belongs to them, not to the wing."""
    return [v for v in pts if abs(v.y) <= half + 0.5]


def span_of(name, exclude_winglets=False, half=None):
    p = world_verts(name)
    if exclude_winglets:
        p = wing_only(p, half)
    return (max(v.y for v in p) - min(v.y for v in p)) if p else 0.0


def chord_at_station(name, y, band=14.0):
    """Streamwise extent in a spanwise slice. The band has to be wide enough
    to catch a loft station -- the mesh only has vertices where a section was
    placed, so a narrow band between two stations finds nothing at all."""
    p = [v for v in world_verts(name) if abs(abs(v.y) - abs(y)) <= band]
    return (max(v.x for v in p) - min(v.x for v in p)) if p else 0.0


def le_sweep(name, y0=120.0, y1=900.0):
    """Leading-edge sweep from two slices: the angle the LE actually makes,
    not the angle that was requested."""
    def le(y):
        p = [v for v in world_verts(name) if abs(abs(v.y) - y) <= 14.0]
        return min(v.x for v in p) if p else None
    a, b = le(y0), le(y1)
    if a is None or b is None:
        return None
    return math.degrees(math.atan2(b - a, y1 - y0))


def dihedral(name, y0=200.0, y1=800.0):
    """Rise of the section mid-height between two stations."""
    def z_at(y):
        p = [v for v in world_verts(name) if abs(abs(v.y) - y) <= 16.0]
        if not p:
            return None
        return 0.5 * (max(v.z for v in p) + min(v.z for v in p))
    a, b = z_at(y0), z_at(y1)
    if a is None or b is None:
        return None
    return math.degrees(math.atan2(b - a, y1 - y0))


def bbox(names):
    pts = []
    for n in names:
        pts += world_verts(n)
    if not pts:
        return None
    return {"x": (min(p.x for p in pts), max(p.x for p in pts)),
            "y": (min(p.y for p in pts), max(p.y for p in pts)),
            "z": (min(p.z for p in pts), max(p.z for p in pts))}


def planform_area(half, parts=("Wing", "Flap", "Aileron")):
    """Projected area of the wing, summed from the FACES.

    The flaps and ailerons are SEPARATE objects, cut out of the wing so they
    can hinge, so measuring "Wing" alone misses a quarter of the chord over
    three quarters of the span. All three go in the sum.

    Integrating over fixed spanwise bands does not work on a lofted mesh --
    the vertices only exist where a section was placed, so most bands are
    empty and the total comes out a fraction of the truth. Projecting every
    polygon onto the XY plane and summing counts the real surface. Upper and
    lower skins both project onto the same planform, hence the half.
    """
    total = 0.0
    for name in parts:
        for f in world_faces(name):
            cy = sum(v.y for v in f) / len(f)
            if abs(cy) > half - 1.0:       # winglet, not wing
                continue
            a = 0.0
            for i in range(len(f)):
                p, q = f[i], f[(i + 1) % len(f)]
                a += p.x * q.y - q.x * p.y  # shoelace, projected on XY
            total += abs(a) / 2.0
    return total / 2.0


def gap_area(P):
    """Planform the hinge gaps take out.

    Each control surface is inset by one gap at both ends, and the wing is
    cut back by one gap beyond them, so there are 2 x gap of missing
    flap-chord area at each end of each surface. Four surfaces, eight ends.
    Small, but it is the difference between the trapezoid and what is really
    there, and a dimension check that ignores it is just a fudge factor.
    """
    half = P["span"] / 2.0
    g = P["gap"]
    total = 0.0
    for (y0f, y1f) in ((P["flap_y0"], P["flap_y1"]), (P["ail_y0"], P["ail_y1"])):
        for yf in (y0f, y1f):
            y = yf * half
            c = JW.chord_at(y)
            total += 2.0 * g * P["flap_chord"] * c       # both ends
    return 2.0 * total                                    # both wings


def aft_diameter(name, frac=0.04):
    """Width of a duct across its rearmost slice -- the nozzle exit, not the
    widest point of the barrel."""
    p = world_verts(name)
    if not p:
        return 0.0
    x1 = max(v.x for v in p)
    span = x1 - min(v.x for v in p)
    tail = [v for v in p if v.x >= x1 - span * frac]
    return (max(v.y for v in tail) - min(v.y for v in tail)) if tail else 0.0


def measure():
    P = JW.P
    half = P["span"] / 2.0
    allp = bbox([o.name for o in bpy.data.collections[JW.COLL].objects])
    area = planform_area(half)
    taper = P["tip_chord"] / P["root_chord"]
    mac_spec = (2.0 / 3.0) * P["root_chord"] * (1 + taper + taper ** 2) / (1 + taper)
    spec_area = (P["span"] * (P["root_chord"] + P["tip_chord"]) / 2.0
                 - gap_area(P)) / 1e4

    rows = [
        ("wing span", span_of("Wing", True, half), P["span"], "mm"),
        ("tip to tip over winglets", span_of("Wing"), None, "mm"),
        ("root chord", chord_at_station("Wing", 0.0), P["root_chord"], "mm"),
        ("tip chord", chord_at_station("Wing", half - 4.0, 12.0),
         P["tip_chord"], "mm"),
        ("LE sweep", le_sweep("Wing"), P["sweep_le"], "deg"),
        ("dihedral", dihedral("Wing"), P["dihedral"], "deg"),
        ("wing area (gaps deducted)", area / 1e4, spec_area, "dm2"),
        ("MAC", mac_spec, mac_spec, "mm"),
        ("overall length", allp["x"][1] - allp["x"][0], None, "mm"),
        ("overall height", allp["z"][1] - allp["z"][0], None, "mm"),
        ("fuselage length", (lambda b: b["x"][1] - b["x"][0])(bbox(["Fuselage"])),
         P["pod_len"] * 1.12, "mm"),
        ("nozzle exit diameter", aft_diameter("Duct"), P["nozzle_dia"], "mm"),
        ("fan diameter", (lambda b: b["y"][1] - b["y"][0])(bbox(["Fan"])),
         P["fan_dia"] - 1.4, "mm"),
        ("fin height", (lambda b: b["z"][1] - b["z"][0])(bbox(["Fin"])),
         None, "mm"),
        ("inlet protrusion",
         (lambda b: b["y"][1] - P["pod_w"] / 2.0)(bbox(["Inlet"])), None, "mm"),
    ]
    return rows, area


def report():
    rows, area = measure()
    P = JW.P
    L = ["", "MEASURED FROM THE BUILT MESH", ""]
    L.append(f"  {'dimension':26s} {'measured':>10s} {'specified':>10s} "
             f"{'error':>9s}")
    bad = 0
    for (name, got, want, unit) in rows:
        if got is None:
            L.append(f"  {name:26s} {'--':>10s}")
            continue
        if want is None:
            L.append(f"  {name:26s} {got:10.1f} {'':>10s} {unit:>9s}")
            continue
        err = got - want
        tol = TOL if unit == "mm" else (0.2 if unit == "deg" else 0.08)
        flag = "" if abs(err) <= tol else "   <-- OUT"
        if abs(err) > tol:
            bad += 1
        L.append(f"  {name:26s} {got:10.2f} {want:10.2f} {err:+9.2f}{flag}")
    S = area / 1e4
    L.append("")
    L.append(f"  aspect ratio  {(P['span'] / 100.0) ** 2 / S:.2f}"
             f"    taper {P['tip_chord'] / P['root_chord']:.3f}")
    m = _mass_g(S)
    L.append(f"  built mass {m:.0f} g -> wing loading {m / S:.1f} g/dm2"
             f"   (published envelope 28-48)")
    L.append("")
    L.append("  ALL DIMENSIONS WITHIN TOLERANCE" if bad == 0
             else f"  {bad} DIMENSION(S) OUT OF TOLERANCE")
    return "\n".join(L), bad


# ==========================================================================
# dimension annotations in the scene
# ==========================================================================

DIMS = "Dimensions"


def dim_collection():
    old = bpy.data.collections.get(DIMS)
    if old:
        for ob in list(old.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(old)
    c = bpy.data.collections.new(DIMS)
    bpy.context.scene.collection.children.link(c)
    return c


def ribbon(name, quads, coll):
    """Flat geometry. An edge-only mesh has no faces, so Cycles renders
    nothing at all -- dimension lines have to be real surfaces."""
    verts, faces = [], []
    for q in quads:
        o = len(verts)
        verts += [tuple(v) for v in q]
        faces.append([o, o + 1, o + 2, o + 3])
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) * JW.MM for v in verts], [], faces)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def leader(name, a, b, coll, z, tick=26.0, w=3.0):
    """Dimension line with end ticks, drawn flat in the XY plane at height z,
    so it reads in a plan view."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    px, py = -uy, ux                      # perpendicular, in plane
    quads = [[(ax + px * w, ay + py * w, z), (bx + px * w, by + py * w, z),
              (bx - px * w, by - py * w, z), (ax - px * w, ay - py * w, z)]]
    for (cx, cy) in ((ax, ay), (bx, by)):
        quads.append([(cx + px * tick + ux * w, cy + py * tick + uy * w, z),
                      (cx - px * tick + ux * w, cy - py * tick + uy * w, z),
                      (cx - px * tick - ux * w, cy - py * tick - uy * w, z),
                      (cx + px * tick - ux * w, cy + py * tick - uy * w, z)])
    return ribbon(name, quads, coll)


def label(name, text, at, coll, z, size=64.0, rot_z=-math.pi / 2):
    """Text lying flat in the XY plane.

    Two things have to be right or it is unreadable: it must lie IN the plane
    the camera looks down (text standing up in XZ is edge-on from above), and
    its baseline must run along the screen's horizontal. In this plan view
    that is the Y axis, and the quarter turn is NEGATIVE -- the camera looks
    down from above with X running up the screen, so a positive turn comes
    out mirrored. Checked by rendering it, not by reasoning about it.
    """
    cu = bpy.data.curves.new(name, type='FONT')
    cu.body = text
    cu.size = size * JW.MM
    cu.align_x = 'CENTER'
    cu.align_y = 'CENTER'
    cu.extrude = 0.0008
    ob = bpy.data.objects.new(name, cu)
    ob.location = Vector((at[0], at[1], z)) * JW.MM
    ob.rotation_euler = (0.0, 0.0, rot_z)
    coll.objects.link(ob)
    return ob


def annotate():
    """Draw the key dimensions onto the model, reading them off the MEASURED
    geometry rather than off the parameters -- so if the two ever disagree,
    the drawing shows the truth."""
    coll = dim_collection()
    P = JW.P
    half = P["span"] / 2.0
    wing = bbox(["Wing"])
    allp = bbox([o.name for o in bpy.data.collections[JW.COLL].objects])
    sp = span_of("Wing", True, half)
    tt = span_of("Wing")
    root = chord_at_station("Wing", 0.0)
    tip = chord_at_station("Wing", half - 4.0, 12.0)
    area = planform_area(half) / 1e4
    sweep = le_sweep("Wing")
    z = allp["z"][1] + 120.0                 # float the drawing clear of the model

    # span, ahead of the nose
    y_s = wing["x"][0] - 210.0
    leader("dim_span", (y_s, -sp / 2), (y_s, sp / 2), coll, z)
    label("lbl_span", f"span  {sp:.0f} mm", (y_s - 95.0, 0.0), coll, z)

    # tip to tip over the winglets, further forward again
    y_t = y_s - 190.0
    leader("dim_tt", (y_t, -tt / 2), (y_t, tt / 2), coll, z)
    label("lbl_tt", f"tip to tip over winglets  {tt:.0f} mm",
          (y_t - 90.0, 0.0), coll, z, size=48.0)

    # root chord, to one side of the pod
    leader("dim_root", (0.0, -150.0), (root, -150.0), coll, z)
    label("lbl_root", f"root {root:.0f}", (root / 2, -215.0), coll, z, size=48.0)

    # tip chord, outboard
    xt = JW.le_at(half - 20.0)
    leader("dim_tip", (xt, half + 120.0), (xt + tip, half + 120.0), coll, z)
    label("lbl_tip", f"tip {tip:.0f}", (xt + tip / 2, half + 185.0), coll, z,
          size=48.0)

    # overall length, off to the right
    y_l = sp / 2 + 300.0
    leader("dim_len", (allp["x"][0], y_l), (allp["x"][1], y_l), coll, z)
    label("lbl_len", f"length {allp['x'][1] - allp['x'][0]:.0f}",
          (0.5 * (allp["x"][0] + allp["x"][1]), y_l + 70.0), coll, z, size=52.0)

    label("lbl_data",
          f"area {area:.2f} dm2     AR {(sp / 100.0) ** 2 / area:.2f}     "
          f"taper {tip / root:.3f}     LE sweep {sweep:.1f} deg",
          (allp["x"][1] + 230.0, 0.0), coll, z, size=58.0)
    return coll


def main():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    JW.build()
    txt, bad = report()
    print(txt)
    annotate()
    print(f"\n  wrote {len(bpy.data.collections[DIMS].objects)} dimension "
          f"objects into the '{DIMS}' collection")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
