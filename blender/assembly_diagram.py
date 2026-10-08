#!/usr/bin/env python3
"""Plan view with the spars, the section joints and the CG drawn on."""
import math, os, sys
import bpy
from mathutils import Vector, Matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW
import print_parts as PP
import measure as M
import render_jetwing as R

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "img", "blender")
CG_X = 231.0          # mm aft of the wing root leading edge


def mat(name, rgb, alpha=1.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*rgb, 1.0)
    b.inputs["Roughness"].default_value = 0.45
    if alpha < 1.0:
        b.inputs["Alpha"].default_value = alpha
        m.blend_method = 'BLEND'
    return m


if __name__ == "__main__":
    R.clear()
    JW.build()
    coll = bpy.data.collections[JW.COLL]
    work = bpy.data.collections.new("ann")
    bpy.context.scene.collection.children.link(work)

    see_through = mat("Ghost", (0.88, 0.89, 0.92), alpha=0.16)
    for ob in coll.objects:
        if ob.type != 'MESH':
            continue
        ob.data.materials.clear()
        ob.data.materials.append(see_through)

    f = PP.SPAR["frac"]
    carbon = mat("Carbon", (0.02, 0.02, 0.03))
    rods = [(-PP.SPAR["A_to"], PP.SPAR["A_to"], PP.SPAR["A_od"], f, "SparA"),
            (PP.SPAR["B_from"], PP.SPAR["B_to"], PP.SPAR["B_od"], f, "SparB_R"),
            (-PP.SPAR["B_to"], -PP.SPAR["B_from"], PP.SPAR["B_od"], f, "SparB_L"),
            (-PP.SPAR["anti_to"], PP.SPAR["anti_to"], PP.SPAR["anti_od"],
             PP.SPAR["anti_frac"], "AntiRotation")]
    for (y0, y1, od, frac, nm) in rods:
        ob = PP.cylinder(PP.spar_point(y0, frac), PP.spar_point(y1, frac),
                         od, nm, work)
        ob.data.materials.append(carbon)

    # section joints
    half = JW.P["span"] / 2.0
    joint = mat("Joint", (0.86, 0.17, 0.12))
    for c in PP.CUTS[1:-1]:
        for sgn in (1.0, -1.0):
            y = sgn * c * half
            a = Vector((JW.le_at(y) - 6.0, y, 40.0))
            b = Vector((JW.le_at(y) + JW.chord_at(y) + 6.0, y, 40.0))
            ob = PP.cylinder(a, b, 6.0, f"joint{c}{sgn:+.0f}", work)
            ob.data.materials.append(joint)

    # centre of gravity
    cg = mat("CG", (0.95, 0.75, 0.10))
    for (a, b) in (((CG_X, -90, 55), (CG_X, 90, 55)),
                   ((CG_X - 90, 0, 55), (CG_X + 90, 0, 55))):
        ob = PP.cylinder(Vector(a), Vector(b), 11.0, f"cg{a[0]}{a[1]}", work)
        ob.data.materials.append(cg)

    for (txt, at, size) in (
            (f"CG  {CG_X:.0f} mm aft of root LE  (318 mm from the nose)",
             (CG_X, 330.0), 44.0),
            ("Spar A  10 x 8 tube, 900 mm, wing joiner", (-150.0, -430.0), 42.0),
            ("Spar B  8 x 6 tube, 500 mm each side", (430.0, 760.0), 42.0),
            ("red lines = printed section joints", (-300.0, -430.0), 38.0)):
        M.label(txt[:18], txt.replace("  ", " "), at, work, 150.0, size=size)
    for ob in work.objects:
        if ob.type == 'FONT':
            ob.data.materials.append(mat("Txt", (0.10, 0.11, 0.13)))

    mn = Vector((1e9,) * 3); mx = Vector((-1e9,) * 3)
    deps = bpy.context.evaluated_depsgraph_get()
    for c in (coll, work):
        for ob in c.objects:
            for p in ob.evaluated_get(deps).bound_box:
                w = ob.matrix_world @ Vector(p)
                for i in range(3):
                    mn[i] = min(mn[i], w[i]); mx[i] = max(mx[i], w[i])
    centre, size = (mn + mx) / 2, (mx - mn).length
    R.camera(centre, size, 180, 89.9, k=1.30)
    R.render(os.path.join(OUT, "assembly-spars.png"), 1700, 1150)
