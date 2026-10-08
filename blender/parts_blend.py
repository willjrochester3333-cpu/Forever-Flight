#!/usr/bin/env python3
"""
Build jetwing-parts.blend: every printable part, in one file, ready to open.

    python3 blender/parts_blend.py
    blender --background --python blender/parts_blend.py

Two collections:

    Assembled     the aircraft, for reference (hidden on open)
    Print Parts   all 33 pieces, each standing in its PRINT orientation,
                  sitting on z = 0, laid out on a grid

and an A1 build volume drawn to one side, so "does it fit" is something you
can see rather than something you have to take on trust.

Select a part, File > Export > STL, tick Selection Only. Or just look at it.
"""

import math
import os
import sys

import bpy
from mathutils import Vector, Matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW
import print_parts as PP

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PITCH = 0.300          # grid spacing, metres
COLS = 7


def stand_up(ob):
    """Rotate a part onto the face it prints on.

    Wing and control sections stand on a spanwise cut face, so the spar bore
    runs up Z and needs no support. Fuselage rings stand on a former face.
    Everything else is already sitting the right way up.
    """
    n = ob.name
    if n.startswith(("wing_", "flap_", "aileron_")):
        ob.data.transform(Matrix.Rotation(math.radians(90.0), 4, 'X'))
    elif n.startswith("fuselage_"):
        ob.data.transform(Matrix.Rotation(math.radians(90.0), 4, 'Y'))
    elif n in ("part_Ducta", "part_Ductb", "part_Motor"):
        ob.data.transform(Matrix.Rotation(math.radians(90.0), 4, 'Y'))


def drop_to_floor(ob):
    lo = min((v.co.z for v in ob.data.vertices), default=0.0)
    cx = sum(v.co.x for v in ob.data.vertices) / max(1, len(ob.data.vertices))
    cy = sum(v.co.y for v in ob.data.vertices) / max(1, len(ob.data.vertices))
    ob.data.transform(Matrix.Translation(Vector((-cx, -cy, -lo))))


def wire_box(name, size, coll, at):
    """A wireframe cube: the build volume."""
    s = size / 2.0
    v = [(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0),
         (-s, -s, size), (s, -s, size), (s, s, size), (-s, s, size)]
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(p) * JW.MM for p in v], e, [])
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.location = at
    ob.display_type = 'WIRE'
    coll.objects.link(ob)
    return ob


def material(name, rgb):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = rgb
    b.inputs["Roughness"].default_value = 0.45
    return m


def label(text, at, coll, size=26.0):
    cu = bpy.data.curves.new(text, type='FONT')
    cu.body = text
    cu.size = size * JW.MM
    cu.align_x = 'CENTER'
    ob = bpy.data.objects.new("lbl_" + text, cu)
    ob.location = at
    coll.objects.link(ob)
    return ob


def main():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)

    PP.main()                      # builds the aircraft and cuts the parts

    src = bpy.data.collections[JW.COLL]
    src.name = "Assembled"
    parts_src = bpy.data.collections["parts"]
    parts_src.name = "Print Parts"
    for c in list(bpy.data.collections):
        if c.name == "work":
            for o in list(c.objects):
                bpy.data.objects.remove(o, do_unlink=True)
            bpy.data.collections.remove(c)

    # Gather by NAME, not by collection: the fin, rudder, inlet, stator and
    # motor are used whole, so they were renamed in place and never moved out
    # of the assembled collection. Collecting by collection quietly lost five
    # parts, which is the kind of thing you only notice by counting.
    pre = ("wing_", "flap_", "aileron_", "fuselage_", "part_")
    parts = [o for o in bpy.data.objects
             if o.type == 'MESH' and o.name.startswith(pre)
             and len(o.data.vertices)]
    parts.sort(key=lambda o: o.name)
    for o in parts:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        parts_src.objects.link(o)
    assert len(parts) == 33, f"expected 33 parts, laid out {len(parts)}"

    mat = material("Part", (0.86, 0.87, 0.90, 1.0))
    for i, ob in enumerate(parts):
        stand_up(ob)
        drop_to_floor(ob)
        ob.location = ((i % COLS - (COLS - 1) / 2.0) * PITCH,
                       -(i // COLS) * PITCH, 0.0)
        ob.rotation_euler = (0.0, 0.0, 0.0)
        ob.data.materials.clear()
        ob.data.materials.append(mat)
        d = PP.dims_mm(ob)
        label(f"{ob.name}  {d[0]:.0f}x{d[1]:.0f}x{d[2]:.0f}",
              ob.location + Vector((0.0, -0.115, 0.0)), parts_src)

    rows = (len(parts) + COLS - 1) // COLS
    wire_box("A1_build_volume_256", PP.BED, parts_src,
             Vector(((COLS / 2.0 + 0.9) * PITCH, -(rows / 2.0) * PITCH, 0.0)))
    label("Bambu Lab A1  256 x 256 x 256",
          Vector(((COLS / 2.0 + 0.9) * PITCH, -(rows / 2.0) * PITCH - 0.17, 0.0)),
          parts_src, size=30.0)

    # The aircraft is for reference; the parts are what you came for. Hide it
    # in BOTH the viewport and renders -- hide_viewport alone leaves it in
    # every render, which is exactly what it did.
    bpy.context.view_layer.layer_collection.children["Assembled"].hide_viewport = True
    for o in src.objects:
        o.hide_render = True
        o.hide_set(True)

    out = os.path.join(ROOT, "blender", "jetwing-parts.blend")
    bpy.ops.wm.save_as_mainfile(filepath=out)
    print(f"\n  {len(parts)} parts laid out, saved {out}")


if __name__ == "__main__":
    main()
