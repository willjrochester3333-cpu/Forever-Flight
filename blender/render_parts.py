#!/usr/bin/env python3
"""
Render the printable parts: the plate layout, and a cutaway of a servo bay.

    python3 blender/render_parts.py
    blender --background --python blender/render_parts.py

These three images are in docs/PRINTING.md and docs/ASSEMBLY.md. They used to
be made by hand each time the cut changed, which meant the documentation
pictures drifted away from the parts -- the published layout showed a 37-part
A1 cut long after the cut had moved on. This script makes them from the cut
itself, so they cannot.
"""

import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW
import print_parts as PP
import parts_blend as PB
import render_jetwing as R

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "img", "blender")


def dim(factor):
    """render_jetwing's lighting is set for a whole aircraft against a pale
    background. Pointed at one part it blows the surface out to flat white,
    which is how a photograph of a servo bay ends up showing no servo bay."""
    for ob in bpy.data.objects:
        if ob.type == 'LIGHT':
            ob.data.energy *= factor


def layout():
    """Every part standing in print orientation, gridded, with the A2L volume
    beside them -- the same scene parts_blend.py saves."""
    PB.main()
    coll = bpy.data.collections["Print Parts"]
    # The default part colour is near-white and so is the background, which
    # renders as a white-on-white puzzle. Darken both the parts and the
    # labels for the photograph only.
    part = PB.material("PartRender", (0.42, 0.46, 0.55, 1.0))
    text = PB.material("LabelRender", (0.12, 0.13, 0.16, 1.0))
    for ob in coll.objects:
        if ob.type == 'MESH':
            ob.data.materials.clear()
            ob.data.materials.append(part)
        elif ob.type == 'FONT':
            ob.data.materials.clear()
            ob.data.materials.append(text)
    mn = Vector((1e9,) * 3)
    mx = Vector((-1e9,) * 3)
    deps = bpy.context.evaluated_depsgraph_get()
    for ob in coll.objects:
        ev = ob.evaluated_get(deps)
        for c in ev.bound_box:
            w = ev.matrix_world @ Vector(c)
            for i in range(3):
                mn[i] = min(mn[i], w[i])
                mx[i] = max(mx[i], w[i])
    centre = (mn + mx) / 2.0
    size = (mx - mn).length
    R.camera(centre, size, az=-80.0, el=52.0, k=1.46)
    dim(0.30)
    R.render(os.path.join(OUT, "parts-blend.png"), 1900, 1250)
    for ob in list(bpy.data.objects):
        if ob.type in ('CAMERA', 'LIGHT'):
            bpy.data.objects.remove(ob, do_unlink=True)
    R.camera(centre, size, az=-90.0, el=88.0, k=1.24)
    dim(0.30)
    R.render(os.path.join(OUT, "print-parts.png"), 1900, 1250)


def servo_bay():
    """One wing section cut across the flap servo station, so the bay, the
    pushrod guide and the loom channel are all visible at once."""
    R.clear()
    JW.build()
    work = bpy.data.collections.new("work")
    bpy.context.scene.collection.children.link(work)
    pc = bpy.data.collections.new("parts")
    bpy.context.scene.collection.children.link(pc)

    half = JW.P["span"] / 2.0
    y0, y1 = PP.CUTS[1] * half, PP.CUTS[2] * half
    ys = PP.SERVO["stations"]["flap"]
    ob = PP.wing_segment(y0, y1, "bay_demo", pc)
    f = PP.SPAR["frac"]
    for (od, a, b) in ((PP.SPAR["A_od"], max(y0, 0.0), min(y1, PP.SPAR["A_to"])),
                       (PP.SPAR["B_od"], max(y0, PP.SPAR["B_from"]),
                        min(y1, PP.SPAR["B_to"]))):
        if b - a < 2.0:
            continue
        c = PP.cylinder(PP.spar_point(a - 14.0, f), PP.spar_point(b + 14.0, f),
                        od + PP.SPAR["fit"], "bore", work)
        PP.boolean(ob, c)
        bpy.data.objects.remove(c, do_unlink=True)
    PP.wire_channel(ob, y0, min(y1, PP.SERVO["wire_to"]), work)
    PP.servo_bay(ob, ys, work)
    hatch = PP.servo_hatch(ys, "hatch_demo", pc)

    # Slice the section open ON the bay centreline, so the cut face shows the
    # bay, the spar bore and the loom channel in section all at once. A
    # photograph of the opening from underneath shows a rectangle and tells
    # you nothing.
    lo, hi = PP.bounds_of(ob)
    keep = PP.box((lo[0] - 40, lo[1] - 40, lo[2] - 40),
                  (hi[0] + 40, ys, hi[2] + 40), "keep", work)
    PP.boolean(ob, keep, 'INTERSECT')
    bpy.data.objects.remove(keep, do_unlink=True)

    mat = PB.material("Part", (0.52, 0.56, 0.64, 1.0))
    hmat = PB.material("Hatch", (0.84, 0.46, 0.22, 1.0))
    for (o, m) in ((ob, mat), (hatch, hmat)):
        o.data.materials.clear()
        o.data.materials.append(m)
    # stand the hatch off below, so it reads as the separate part it is
    hatch.location = Vector((0.0, 0.0, -0.045))

    for o in list(work.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    src = bpy.data.collections.get(JW.COLL)
    if src:
        for o in list(src.objects):
            bpy.data.objects.remove(o, do_unlink=True)

    # From outboard and below: the cut face is looking out along +y and the
    # bay opens downwards, so that is the one direction that shows both.
    target = PP.skin_point(ys, PP.SERVO["chord_frac"], lower=True) * JW.MM
    target = Vector((target.x, target.y - 0.030, target.z + 0.004))
    R.camera(target, 0.30, az=62.0, el=-24.0, k=1.00)
    dim(0.11)
    ld = bpy.data.lights.new("Under", 'AREA')
    ld.energy = 45.0
    ld.size = 0.8
    lo = bpy.data.objects.new("Under", ld)
    lo.location = target + Vector((0.10, 0.22, -0.26))
    lo.rotation_euler = (target - lo.location).normalized().to_track_quat(
        '-Z', 'Y').to_euler()
    bpy.context.scene.collection.objects.link(lo)
    R.render(os.path.join(OUT, "servo-bay.png"), 1600, 1050)


def main():
    servo_bay()
    layout()
    print("\n  wrote parts-blend.png, print-parts.png, servo-bay.png to %s/" % OUT)


if __name__ == "__main__":
    main()
