#!/usr/bin/env python3
"""Render the dimensioned plan and side views."""
import math, os, sys
import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW
import measure as M
import render_jetwing as R

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "img", "blender")

if __name__ == "__main__":
    R.clear()
    JW.build()
    txt, bad = M.report()
    print(txt)
    M.annotate()
    # dimension lines read best flat-on and unshaded
    for ob in bpy.data.collections[M.DIMS].objects:
        mat = bpy.data.materials.get("Dim") or bpy.data.materials.new("Dim")
        mat.use_nodes = True
        b = mat.node_tree.nodes.get("Principled BSDF")
        b.inputs["Base Color"].default_value = (0.08, 0.09, 0.11, 1.0)
        if "Emission Color" in b.inputs:
            b.inputs["Emission Color"].default_value = (0.08, 0.09, 0.11, 1.0)
            b.inputs["Emission Strength"].default_value = 1.0
        ob.data.materials.append(mat)

    # frame the DIMENSIONS too, not just the aircraft -- they sit well
    # outside it and were falling off the edge of the picture
    mn = Vector((1e9,) * 3); mx = Vector((-1e9,) * 3)
    deps = bpy.context.evaluated_depsgraph_get()
    for cname in (JW.COLL, M.DIMS):
        for ob in bpy.data.collections[cname].objects:
            ev = ob.evaluated_get(deps)
            for c in ev.bound_box:
                w = ev.matrix_world @ Vector(c)
                for i in range(3):
                    mn[i] = min(mn[i], w[i]); mx[i] = max(mx[i], w[i])
    centre, size = (mn + mx) / 2, (mx - mn).length
    for (nm, az, el) in (("jetwing-dims-plan", 180, 89.9),):
        for ob in list(bpy.data.objects):
            if ob.type in ('CAMERA', 'LIGHT'):
                bpy.data.objects.remove(ob, do_unlink=True)
        R.camera(centre, size, az, el, k=1.34)
        R.render(os.path.join(OUT, nm + ".png"), 1700, 1150)
    print("done")
