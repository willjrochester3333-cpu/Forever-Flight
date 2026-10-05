#!/usr/bin/env python3
"""
A basic RC plane, built in Blender.

HOW TO RUN IT

  In Blender:  Scripting tab -> Open -> this file -> Run Script (Alt+P)
  Headless:    blender --background --python blender/basic_plane.py

Everything is driven by the P dictionary below. Change a number, run it
again -- the script clears its own objects first, so you can iterate without
piling up duplicates.

WHAT YOU GET

  a conventional high-wing trainer: fuselage, wing with dihedral, tailplane,
  fin, undercarriage, nose spinner and propeller disc. Separate named objects
  in a "Plane" collection, each with a material, mirrored where it should be,
  and a Subdivision/Shade-Smooth pass on the curved bodies.

It is deliberately simple. Everything is built from lofted sections, so if
you want an aerofoil instead of a rounded plate, swap the section function
and the rest still works.
"""

import math

import bpy
import bmesh
from mathutils import Vector

# ==========================================================================
# parameters -- all millimetres, converted to metres on the way in
# ==========================================================================

P = {
    "span":         1200.0,
    "root_chord":    200.0,
    "tip_chord":     150.0,
    "dihedral":        4.0,   # deg
    "wing_x":        170.0,   # leading edge aft of the nose
    "wing_z":         34.0,   # above the fuselage centreline (high wing)
    "wing_thick":      0.12,  # fraction of chord

    "fus_len":       900.0,
    "fus_w":          78.0,
    "fus_h":          96.0,

    "tail_span":     380.0,
    "tail_chord":     98.0,
    "tail_x":        760.0,

    "fin_h":         150.0,
    "fin_root":      130.0,
    "fin_tip":        70.0,
    "fin_sweep":      32.0,   # deg
    "fin_x":         750.0,

    "prop_dia":      254.0,   # 10 inch
    "spinner":        40.0,

    "wheel_dia":      54.0,
    "gear_x":        250.0,
    "gear_track":    170.0,
    "gear_drop":     110.0,

    "smooth": True,
}

MM = 0.001
COLL = "Plane"

COLOURS = {
    "Fuselage":  (0.85, 0.86, 0.88, 1.0),
    "Wing":      (0.90, 0.91, 0.93, 1.0),
    "Tail":      (0.82, 0.24, 0.20, 1.0),
    "Trim":      (0.14, 0.32, 0.62, 1.0),
    "Rubber":    (0.06, 0.06, 0.07, 1.0),
    "Glass":     (0.30, 0.45, 0.55, 0.35),
}


# ==========================================================================
# scene helpers
# ==========================================================================

def collection():
    """A clean collection to build into, wiping anything from a previous run."""
    old = bpy.data.collections.get(COLL)
    if old:
        for ob in list(old.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(old)
    c = bpy.data.collections.new(COLL)
    bpy.context.scene.collection.children.link(c)
    return c


def material(name):
    m = bpy.data.materials.get(name)
    if m is None:
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get("Principled BSDF")
        r, g, b, a = COLOURS[name]
        bsdf.inputs["Base Color"].default_value = (r, g, b, 1.0)
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.45
        if a < 1.0:
            m.blend_method = "BLEND"
            if "Alpha" in bsdf.inputs:
                bsdf.inputs["Alpha"].default_value = a
    return m


def mesh_from(name, verts, faces, mat, coll, smooth=False, mirror=False):
    """Build an object from vertex/face lists. Optionally mirror in Y."""
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) * MM for v in verts], [], faces)
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.data.materials.append(material(mat))
    coll.objects.link(ob)

    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.to_mesh(me)
    bm.free()

    if mirror:
        m = ob.modifiers.new("Mirror", "MIRROR")
        m.use_axis = (False, True, False)
        m.use_clip = True
    if smooth and P["smooth"]:
        sub = ob.modifiers.new("Subdivision", "SUBSURF")
        sub.levels = 1
        sub.render_levels = 2
        for poly in me.polygons:
            poly.use_smooth = True
    return ob


def loft(sections, cap_start=True, cap_end=True):
    """Join a list of equal-length rings into a tube."""
    verts, faces = [], []
    n = len(sections[0])
    for ring in sections:
        verts.extend(ring)
    for i in range(len(sections) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n):
            k = (j + 1) % n
            faces.append([a + j, a + k, b + k, b + j])
    if cap_start:
        faces.append(list(range(n - 1, -1, -1)))
    if cap_end:
        o = (len(sections) - 1) * n
        faces.append([o + j for j in range(n)])
    return verts, faces


def ellipse(w, h, npts=20, cx=0.0, cz=0.0):
    return [(0.0, cx + w / 2 * math.cos(2 * math.pi * i / npts),
             cz + h / 2 * math.sin(2 * math.pi * i / npts))
            for i in range(npts)]


def plate(chord, thick, npts=16):
    """A rounded plate section -- a crude aerofoil, good enough to look right
    and simple to swap out for a real one."""
    pts = []
    for i in range(npts + 1):
        x = i / npts
        t = thick * chord * math.sin(math.pi * x) ** 0.8
        pts.append((x * chord, t / 2))
    for i in range(npts, -1, -1):
        x = i / npts
        t = thick * chord * math.sin(math.pi * x) ** 0.8
        pts.append((x * chord, -t / 2))
    return pts


# ==========================================================================
# parts
# ==========================================================================

def fuselage(coll):
    L, w, h = P["fus_len"], P["fus_w"], P["fus_h"]
    #   x fraction, width fraction, height fraction, centre rise
    stations = [(0.00, 0.22, 0.26, 0.02), (0.05, 0.62, 0.70, 0.04),
                (0.14, 0.92, 0.96, 0.04), (0.28, 1.00, 1.00, 0.02),
                (0.44, 0.94, 0.92, 0.00), (0.60, 0.76, 0.74, 0.00),
                (0.76, 0.56, 0.54, 0.01), (0.90, 0.38, 0.38, 0.02),
                (1.00, 0.16, 0.18, 0.03)]
    secs = []
    for (fx, fw, fh, fz) in stations:
        ring = ellipse(w * fw, h * fh, cz=fz * h)
        secs.append([(fx * L, p[1], p[2]) for p in ring])
    v, f = loft(secs)
    return mesh_from("Fuselage", v, f, "Fuselage", coll, smooth=True)


def canopy(coll):
    L, w, h = P["fus_len"], P["fus_w"], P["fus_h"]
    x0, x1 = 0.17 * L, 0.42 * L
    secs = []
    for i in range(7):
        t = i / 6.0
        x = x0 + (x1 - x0) * t
        prof = math.sin(math.pi * (0.15 + 0.7 * t))
        ring = ellipse(w * 0.62 * prof, h * 0.42 * prof, cz=h * 0.46)
        secs.append([(x, p[1], p[2]) for p in ring])
    v, f = loft(secs)
    return mesh_from("Canopy", v, f, "Glass", coll, smooth=True)


def panel(name, span, root, tip, x0, z0, thick, dihedral, sweep, mat, coll,
          vertical=False, mirror=True):
    """One lifting surface, built as a loft from root to tip."""
    secs = []
    for i in range(2):
        t = float(i)
        c = root + (tip - root) * t
        y = span / 2.0 * t
        off = y * math.tan(math.radians(dihedral))
        xle = x0 + y * math.tan(math.radians(sweep))
        prof = plate(c, thick)
        ring = []
        for (px, pz) in prof:
            if vertical:
                ring.append((xle + px, pz, z0 + y))
            else:
                ring.append((xle + px, y, z0 + off + pz))
        secs.append(ring)
    v, f = loft(secs)
    return mesh_from(name, v, f, mat, coll, smooth=False, mirror=mirror)


def strut(a, b, r, npts=10):
    """A round tube between two points, with rings perpendicular to the axis."""
    ax = Vector(b) - Vector(a)
    L = ax.length
    ax = ax / L
    u = Vector((1.0, 0.0, 0.0))
    if abs(ax.dot(u)) > 0.9:
        u = Vector((0.0, 1.0, 0.0))
    v = ax.cross(u).normalized()
    u = v.cross(ax).normalized()
    secs = []
    for end in (Vector(a), Vector(b)):
        ring = []
        for i in range(npts):
            t = 2 * math.pi * i / npts
            pnt = end + (u * math.cos(t) + v * math.sin(t)) * r
            ring.append((pnt.x, pnt.y, pnt.z))
        secs.append(ring)
    return loft(secs)


def propeller(coll):
    """Spinner plus two real blades: tapered, pitched plates lofted along the
    radius. A blade at angle th lies along the tangential direction
    (0, -sin th, cos th); pitch tilts the chord into x."""
    R = P["prop_dia"] / 2.0
    sp = P["spinner"]
    z0 = 0.04 * P["fus_h"]
    x0 = -sp * 0.35

    secs = []
    for i in range(9):
        t = i / 8.0
        rr = sp / 2.0 * math.sqrt(max(1e-4, 1.0 - t * t))
        secs.append([(-sp * 1.15 * t, p[1], z0 + p[2])
                     for p in ellipse(rr * 2, rr * 2, npts=16)])
    v, f = loft(secs, cap_start=False)
    mesh_from("Spinner", v, f, "Trim", coll, smooth=True)

    pitch_in = 6.0 * 25.4            # a 10x6 propeller
    verts, faces = [], []
    for b in range(2):
        th = math.radians(90.0 + 180.0 * b)
        st, ct = math.sin(th), math.cos(th)
        base = len(verts)
        stations = [(sp * 0.42, 16.0), (R * 0.35, 26.0), (R * 0.62, 28.0),
                    (R * 0.85, 22.0), (R, 6.0)]
        for (rr, ch) in stations:
            beta = math.atan2(pitch_in, 2 * math.pi * max(rr, 12.0))
            sb, cb = math.sin(beta), math.cos(beta)
            thick = 1.6
            for side in (-1.0, 1.0):
                for u in (-ch / 2.0, ch / 2.0):
                    nx, ny, nz = cb, sb * st, -sb * ct
                    verts.append((x0 + u * sb + side * thick / 2 * nx,
                                  rr * ct - u * cb * st + side * thick / 2 * ny,
                                  z0 + rr * st + u * cb * ct + side * thick / 2 * nz))
        n = len(stations)
        for i in range(n - 1):
            a0 = base + i * 4
            a1 = base + (i + 1) * 4
            faces.append([a0 + 0, a0 + 1, a1 + 1, a1 + 0])      # one face
            faces.append([a0 + 2, a1 + 2, a1 + 3, a0 + 3])      # the other
            faces.append([a0 + 0, a1 + 0, a1 + 2, a0 + 2])      # edges
            faces.append([a0 + 1, a0 + 3, a1 + 3, a1 + 1])
        faces.append([base + 0, base + 2, base + 3, base + 1])
        e = base + (n - 1) * 4
        faces.append([e + 1, e + 3, e + 2, e + 0])
    mesh_from("Propeller", verts, faces, "Trim", coll)


def gear(coll):
    d, drop, track = P["wheel_dia"], P["gear_drop"], P["gear_track"] / 2.0
    x = P["gear_x"]
    z0 = -0.40 * P["fus_h"]
    top = (x, 0.22 * track, z0)
    bot = (x, track, z0 - drop)
    v, f = strut(top, bot, 5.0)
    mesh_from("GearLeg", v, f, "Trim", coll, mirror=True)

    # wheel: a short cylinder on a Y axis, centred on the bottom of the leg
    secs = []
    for s in (-6.0, 6.0):
        ring = []
        for i in range(18):
            a = 2 * math.pi * i / 18
            ring.append((bot[0] + d / 2 * math.cos(a), bot[1] + s,
                         bot[2] + d / 2 * math.sin(a)))
        secs.append(ring)
    v, f = loft(secs)
    mesh_from("Wheel", v, f, "Rubber", coll, smooth=True, mirror=True)

    # tail skid, so it sits on three points
    L = P["fus_len"]
    v, f = strut((0.93 * L, 0.0, -0.10 * P["fus_h"]),
                 (0.99 * L, 0.0, z0 - drop * 0.42), 4.0)
    mesh_from("TailSkid", v, f, "Trim", coll)


# ==========================================================================
# build
# ==========================================================================

def build():
    coll = collection()
    fuselage(coll)
    canopy(coll)
    panel("Wing", P["span"], P["root_chord"], P["tip_chord"],
          P["wing_x"], P["wing_z"], P["wing_thick"], P["dihedral"], 2.0,
          "Wing", coll)
    panel("Tailplane", P["tail_span"], P["tail_chord"], P["tail_chord"] * 0.75,
          P["tail_x"], 0.10 * P["fus_h"], 0.10, 0.0, 8.0, "Tail", coll)
    panel("Fin", P["fin_h"] * 2, P["fin_root"], P["fin_tip"],
          P["fin_x"], 0.16 * P["fus_h"], 0.10, 0.0, P["fin_sweep"],
          "Tail", coll, vertical=True, mirror=False)
    propeller(coll)
    gear(coll)

    # centre the whole thing on the origin
    for ob in coll.objects:
        ob.location.x -= P["fus_len"] * MM * 0.5
    print(f"built {len(coll.objects)} objects in collection '{COLL}'")
    return coll


if __name__ == "__main__":
    build()
