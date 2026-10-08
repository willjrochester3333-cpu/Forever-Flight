#!/usr/bin/env python3
"""Build the Jetwing, light it, render previews, save the .blend."""
import math, os, sys
import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jetwing as JW

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "img", "blender")
os.makedirs(OUT, exist_ok=True)


def clear():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)


def extent():
    mn = Vector((1e9,) * 3); mx = Vector((-1e9,) * 3)
    deps = bpy.context.evaluated_depsgraph_get()
    for ob in bpy.data.collections[JW.COLL].objects:
        ev = ob.evaluated_get(deps)
        for c in ev.bound_box:
            w = ev.matrix_world @ Vector(c)
            for i in range(3):
                mn[i] = min(mn[i], w[i]); mx[i] = max(mx[i], w[i])
    return (mn + mx) / 2, (mx - mn).length


def camera(centre, size, az, el, k=1.22):
    cd = bpy.data.cameras.new("Cam"); cd.lens = 52
    cam = bpy.data.objects.new("Cam", cd)
    bpy.context.scene.collection.objects.link(cam)
    a, e = math.radians(az), math.radians(el)
    cam.location = centre + Vector((size * k * math.cos(e) * math.cos(a),
                                    size * k * math.cos(e) * math.sin(a),
                                    size * k * math.sin(e)))
    cam.rotation_euler = (centre - cam.location).normalized().to_track_quat('-Z', 'Y').to_euler()
    bpy.context.scene.camera = cam
    for (nm, loc, en) in (("Key", (2, -3, 4), 1200), ("Fill", (-3, -1, 1.5), 400),
                          ("Rim", (1, 3, 2), 600)):
        ld = bpy.data.lights.new(nm, 'AREA'); ld.energy = en; ld.size = 5.0
        lo = bpy.data.objects.new(nm, ld)
        lo.location = centre + Vector(loc) * size
        lo.rotation_euler = (centre - lo.location).normalized().to_track_quat('-Z', 'Y').to_euler()
        bpy.context.scene.collection.objects.link(lo)


def render(path, w=1500, h=950):
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = w, h
    if sc.world is None:
        sc.world = bpy.data.worlds.new("W")
    sc.world.use_nodes = True
    bg = sc.world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.97, 0.96, 0.95, 1.0)
    sc.render.engine = "CYCLES"          # EEVEE needs a display; this does not
    sc.cycles.device = "CPU"
    sc.cycles.samples = 40
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("wrote", path)


if __name__ == "__main__":
    clear()
    JW.build()
    JW.set_controls()
    centre, size = extent()
    for (nm, az, el) in (("jetwing-iso", 128, 24), ("jetwing-plan", 180, 88),
                         ("jetwing-side", 90, 2), ("jetwing-rear", 35, 18)):
        for ob in list(bpy.data.objects):
            if ob.type in ('CAMERA', 'LIGHT'):
                bpy.data.objects.remove(ob, do_unlink=True)
        camera(centre, size, az, el)
        render(os.path.join(OUT, nm + ".png"))
    # and one with the surfaces deflected, to show the hinges really work
    JW.set_controls(flap=40.0, aileron=-32.0, rudder=22.0)
    for ob in list(bpy.data.objects):
        if ob.type in ('CAMERA', 'LIGHT'):
            bpy.data.objects.remove(ob, do_unlink=True)
    camera(centre, size, 128, 24)
    render(os.path.join(OUT, "jetwing-crow.png"))

    JW.set_controls()
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT, "blender", "jetwing.blend"))
    print("saved blender/jetwing.blend")
