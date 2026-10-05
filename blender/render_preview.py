#!/usr/bin/env python3
"""Build the plane, set up a camera and lights, render previews, save a .blend.

    python3 blender/render_preview.py          (uses the pip-installed bpy)
    blender --background --python blender/render_preview.py
"""
import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import basic_plane as BP

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "img", "blender")
os.makedirs(OUT, exist_ok=True)


def clear_scene():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)


def frame_all():
    mn = Vector((1e9, 1e9, 1e9))
    mx = Vector((-1e9, -1e9, -1e9))
    deps = bpy.context.evaluated_depsgraph_get()
    for ob in bpy.data.collections[BP.COLL].objects:
        ev = ob.evaluated_get(deps)
        for c in ev.bound_box:
            w = ev.matrix_world @ Vector(c)
            for i in range(3):
                mn[i] = min(mn[i], w[i])
                mx[i] = max(mx[i], w[i])
    return (mn + mx) / 2.0, (mx - mn).length


def setup(centre, size, az, el, dist_k=1.5):
    cam_d = bpy.data.cameras.new("Cam")
    cam_d.lens = 70
    cam = bpy.data.objects.new("Cam", cam_d)
    bpy.context.scene.collection.objects.link(cam)
    r = size * dist_k
    a, e = math.radians(az), math.radians(el)
    cam.location = centre + Vector((r * math.cos(e) * math.cos(a),
                                    r * math.cos(e) * math.sin(a),
                                    r * math.sin(e)))
    d = (centre - cam.location).normalized()
    cam.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    bpy.context.scene.camera = cam

    for (key, loc, energy) in (("Key", (2, -3, 4), 900),
                               ("Fill", (-3, -1, 1.5), 300),
                               ("Rim", (1, 3, 2), 400)):
        ld = bpy.data.lights.new(key, 'AREA')
        ld.energy = energy
        ld.size = 4.0
        lo = bpy.data.objects.new(key, ld)
        lo.location = centre + Vector(loc) * size
        dd = (centre - lo.location).normalized()
        lo.rotation_euler = dd.to_track_quat('-Z', 'Y').to_euler()
        bpy.context.scene.collection.objects.link(lo)
    return cam


def render(path, w=1400, h=900):
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.film_transparent = False
    sc.world = bpy.data.worlds.new("W") if sc.world is None else sc.world
    sc.world.use_nodes = True
    bg = sc.world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.97, 0.96, 0.95, 1.0)
        bg.inputs[1].default_value = 1.0
    # EEVEE needs a GPU/EGL, which a headless container has not got.
    # Cycles on CPU renders the same scene with no display at all.
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 48
    sc.cycles.use_denoising = True
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("wrote", path, "engine", sc.render.engine)


if __name__ == "__main__":
    clear_scene()
    BP.build()
    centre, size = frame_all()
    for (name, az, el) in (("plane-iso", 125, 22), ("plane-side", 90, 2),
                           ("plane-front", 180, 2), ("plane-plan", 180, 88)):
        for ob in list(bpy.data.objects):
            if ob.type in ('CAMERA', 'LIGHT'):
                bpy.data.objects.remove(ob, do_unlink=True)
        setup(centre, size, az, el)
        render(os.path.join(OUT, name + ".png"))
    blend = os.path.join(ROOT, "blender", "basic_plane.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend)
    print("saved", blend)
