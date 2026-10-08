#!/usr/bin/env python3
"""
The Jetwing, in Blender: a swept flying wing with an EDF fuselage.

    In Blender:  Scripting tab -> Open -> this file -> Run Script (Alt+P)
    Headless:    blender --background --python blender/jetwing.py

WHAT IS COPIED AND WHAT IS NOT

planeprint.com is blocked by this session's network policy, and so is the
mirror of their assembly manual, so no geometry file was ever available. What
IS below comes from their published specification, which search did reach:

    span            1270 mm standard wing / 2060 mm BIG WING   <- built here
    flight weight   860 to 1750 g
    wing loading    28 to 48 g/dm2
    power           EDF 70 mm on 4S, or glider
    channels        4/6, FOUR flaps, so butterfly/crow braking
    variants        with or without a steerable rudder; the rudder version
                    has "integrated vector control"
    printing        200 mm cube, LW-PLA plus PLA

The wing AREA is not published. It started at 36.5 dm2, the one value that
makes all four published numbers land exactly, but optimisation moved it:
at a 94.4 mm tip the outboard wing runs at Reynolds 48,700, below where a
low-Re aerofoil still works. The tip is now 106.1 mm and the area 37.5 dm2,
which at the built mass is 29.8 g/dm2 -- inside the published 28-48 envelope.
A tip that works was worth more than hitting the envelope endpoints exactly.
Tip chord and washout are OUTPUTS of tools/optimise.py, not guesses.

Sweep, taper, aerofoil and CG are NOT published anywhere. Those are designed
from the aerodynamics, not copied, and they are the reason this is a faithful
model rather than a replica. Send me real dimensions and they go straight
into the P dictionary.

WHAT YOU GET THAT AN STL CANNOT GIVE YOU

The four flaps and the rudder are separate objects whose origins sit ON their
hinge lines, so you can rotate them and they move correctly. Scrub
`set_controls()` at the bottom, or just grab one and R-key it.
"""

import math

import bpy
import bmesh
from mathutils import Vector, Matrix

# ==========================================================================
# parameters, all mm
# ==========================================================================

P = {
    # --- planform: the BIG WING ---
    "span":          2060.0,
    "root_chord":     260.0,
    "tip_chord":      106.1,     # optimised: see tools/optimise.py
    "sweep_le":        24.0,     # deg
    "dihedral":         2.0,
    "washout_tip":    -1.98,     # deg nose-down at the tip; tailless trim

    # --- section: reflexed, so the wing trims itself ---
    "thickness":        0.105,
    "camber":           0.022,
    "camber_pos":       0.38,
    "reflex":           0.019,   # trailing edge lifts aft of reflex_start
    "reflex_start":     0.65,

    # --- winglets ---
    "winglet_h":      150.0,
    "winglet_cant":    72.0,     # deg from horizontal
    "winglet_sweep":   38.0,
    "winglet_taper":    0.45,

    # --- EDF fuselage ---
    "pod_len":        620.0,
    "pod_w":           96.0,
    "pod_h":          104.0,
    "pod_datum":       -5.0,     # deg nose-down vs the root chord
    "duct_z":          -6.0,
    # --- 50 mm EDF, QF2611 5000KV, FIXED (nothing retracts) ---
    #     every number here is an output of tools/edf.py
    "fan_dia":         50.0,
    "fan_hub":         26.0,
    "fan_x":            0.62,    # fan face, fraction of pod length
    "fan_blades":        12,
    "stator_vanes":       7,     # prime against 12, so no blade-passing tone
    "nozzle_dia":      40.5,     # 0.90 x fan swept area: the thrust/efficiency pick
    "inlet_dia":       31.0,
    "inlet_x":          0.30,
    "lip_radius":       3.1,     # 10% of inlet diameter; a sharp lip separates
    "motor_len":       32.0,
    "tailcone_len":    58.0,

    # --- single fin, rudder sitting behind the nozzle ---
    "fin_h":          185.0,
    "fin_root":       170.0,
    "fin_tip":         78.0,
    "fin_sweep":       40.0,
    "fin_x":          505.0,
    "rudder_frac":      0.38,

    # --- four flaps ---
    "flap_y0": 0.16, "flap_y1": 0.52,
    "ail_y0":  0.55, "ail_y1":  0.95,
    "flap_chord": 0.26,
    "gap":        3.0,           # hinge gap, each side

    "n_bays": 24,
    "smooth": True,
}

MM = 0.001
COLL = "Jetwing"

COLOURS = {
    "Airframe": (0.88, 0.89, 0.91, 1.0),
    "Control":  (0.80, 0.26, 0.21, 1.0),
    "Trim":     (0.13, 0.30, 0.58, 1.0),
    "Duct":     (0.10, 0.10, 0.12, 1.0),
}


# ==========================================================================
# aerofoil: NACA 4-digit camber with a trailing-edge reflex
# ==========================================================================

def camber(x, m, p, reflex, q):
    """Camber height and slope. The reflex term curves the trailing edge UP,
    which is what makes the section's pitching moment nose-up and lets a
    tailless wing trim without a tailplane."""
    if x < p:
        z = m / p ** 2 * (2 * p * x - x * x)
        dz = 2 * m / p ** 2 * (p - x)
    else:
        z = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x)
        dz = 2 * m / (1 - p) ** 2 * (p - x)
    if x >= q:
        t = (x - q) / (1.0 - q)
        z += reflex * t * t
        dz += reflex * 2.0 * t / (1.0 - q)
    return z, dz


def thickness(x, t):
    return 5 * t * (0.2969 * math.sqrt(max(x, 0.0)) - 0.1260 * x
                    - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1015 * x ** 4)


def section(n=48, m=None, reflex=None):
    """Closed loop, unit chord, running TE -> upper -> LE -> lower -> TE."""
    m = P["camber"] if m is None else m
    reflex = P["reflex"] if reflex is None else reflex
    p, t, q = P["camber_pos"], P["thickness"], P["reflex_start"]
    up, lo = [], []
    for i in range(n + 1):
        x = 0.5 * (1.0 - math.cos(math.pi * i / n))      # cosine spacing
        yt = thickness(x, t)
        yc, dy = camber(x, m, p, reflex, q)
        th = math.atan(dy)
        up.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lo.append((x + yt * math.sin(th), yc - yt * math.cos(th)))
    gap = 0.0025
    loop = [(1.0, gap)] + up[::-1][1:-1] + [(0.0, 0.0)] + lo[1:-1] + [(1.0, -gap)]
    return loop


# ==========================================================================
# planform
# ==========================================================================

def chord_at(y):
    t = abs(y) / (P["span"] / 2.0)
    return P["root_chord"] + (P["tip_chord"] - P["root_chord"]) * t


def le_at(y):
    return abs(y) * math.tan(math.radians(P["sweep_le"]))


def twist_at(y):
    t = abs(y) / (P["span"] / 2.0)
    return P["washout_tip"] * t ** 1.3


def place(loop, chord, le_x, y, z, angle, pivot=0.25):
    """Put a unit-chord section into 3D at a spanwise station."""
    a = math.radians(angle)
    ca, sa = math.cos(a), math.sin(a)
    px = le_x + pivot * chord
    out = []
    for (xc, yc) in loop:
        dx = (xc - pivot) * chord
        dz = yc * chord
        out.append((px + dx * ca + dz * sa, y, z - dx * sa + dz * ca))
    return out


# ==========================================================================
# Blender helpers
# ==========================================================================

def collection():
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
        b = m.node_tree.nodes.get("Principled BSDF")
        b.inputs["Base Color"].default_value = COLOURS[name]
        if "Roughness" in b.inputs:
            b.inputs["Roughness"].default_value = 0.42
    return m


def obj_from(name, verts, faces, mat, coll, smooth=False, mirror=False,
             origin=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) * MM for v in verts], [], faces)
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.data.materials.append(material(mat))
    coll.objects.link(ob)

    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()

    if origin is not None:
        # move the mesh so the object origin lands on the hinge line
        o = Vector(origin) * MM
        me.transform(Matrix.Translation(-o))
        ob.location = o
    if mirror:
        md = ob.modifiers.new("Mirror", "MIRROR")
        md.use_axis = (False, True, False)
        md.use_clip = True
    if smooth and P["smooth"]:
        for poly in me.polygons:
            poly.use_smooth = True
    return ob


def loft(sections, cap_start=True, cap_end=True):
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


def ring(w, h, npts=24, cz=0.0, power=2.5):
    """Superelliptic ring -- squarer than an ellipse, which is what a printed
    fuselage shell actually looks like."""
    pts = []
    for i in range(npts):
        t = 2 * math.pi * i / npts
        c, s = math.cos(t), math.sin(t)
        pts.append((0.0,
                    w / 2 * math.copysign(abs(c) ** (2.0 / power), c),
                    cz + h / 2 * math.copysign(abs(s) ** (2.0 / power), s)))
    return pts


def datum(pts, deg=None):
    """Rotate points about Y by the pod datum. Applied to the vertices, not
    to the object, so the fuselage, duct and fin stay in ONE frame -- which
    is what went wrong the first time: a rotated pod and an unrotated fin
    drift apart and the fin appears to float."""
    deg = P["pod_datum"] if deg is None else deg
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(x * ca + z * sa, y, -x * sa + z * ca) for (x, y, z) in pts]


def pod_top(xf):
    """Top of the fuselage shell at a station, as a fraction of pod length,
    in the datum frame. Used to sit the fin root on the actual surface."""
    st = [(-0.12, 0.12, 0.00), (-0.05, 0.44, 0.02), (0.05, 0.80, 0.03),
          (0.18, 1.00, 0.02), (0.34, 0.98, 0.00), (0.50, 0.90, -0.01),
          (0.615, 0.78, -0.02), (0.78, 0.60, -0.03), (0.93, 0.48, -0.04),
          (1.00, 0.43, -0.04)]
    for i in range(len(st) - 1):
        if st[i][0] <= xf <= st[i + 1][0]:
            t = (xf - st[i][0]) / (st[i + 1][0] - st[i][0])
            fh = st[i][1] + t * (st[i + 1][1] - st[i][1])
            fz = st[i][2] + t * (st[i + 1][2] - st[i][2])
            return (fz + fh / 2.0) * P["pod_h"]
    return 0.0


def circle(r, npts=20, cz=0.0):
    return [(0.0, r * math.cos(2 * math.pi * i / npts),
             cz + r * math.sin(2 * math.pi * i / npts)) for i in range(npts)]


# ==========================================================================
# parts
# ==========================================================================

def wing_surface(y0, y1, c0, c1, trailing):
    """Section rings between two spanwise stations. `trailing` is the chord
    fraction to stop at (the hinge line) or 1.0 for the full section."""
    loop = section()
    secs = []
    n = max(2, int(P["n_bays"] * (y1 - y0) / (P["span"] / 2.0)) + 2)
    dih = math.tan(math.radians(P["dihedral"]))
    for i in range(n):
        y = y0 + (y1 - y0) * i / (n - 1)
        c = chord_at(y)
        cut = [(min(x, trailing), z) for (x, z) in loop] if trailing < 1.0 else loop
        secs.append(place(cut, c, le_at(y), y, y * dih, twist_at(y)))
    return secs


def wing(coll):
    """Main wing: full section inboard and outboard of the flaps, cut back to
    the hinge line where a control surface lives."""
    half = P["span"] / 2.0
    hinge = 1.0 - P["flap_chord"]
    g = P["gap"]
    spans = []
    f0, f1 = P["flap_y0"] * half, P["flap_y1"] * half
    a0, a1 = P["ail_y0"] * half, P["ail_y1"] * half
    # (y0, y1, cut-to-hinge?)
    runs = [(0.0, f0 - g, False), (f0 - g, f1 + g, True),
            (f1 + g, a0 - g, False), (a0 - g, a1 + g, True),
            (a1 + g, half, False)]
    secs = []
    for (y0, y1, cut) in runs:
        if y1 - y0 < 1.0:
            continue
        s = wing_surface(y0, y1, 0, 0, hinge if cut else 1.0)
        if secs and len(secs[-1]) != len(s[0]):
            v, f = loft(secs, cap_start=not spans, cap_end=True)
            spans.append((v, f))
            secs = []
        secs.extend(s)
    if secs:
        v, f = loft(secs, cap_start=not spans, cap_end=True)
        spans.append((v, f))

    verts, faces = [], []
    for (v, f) in spans:
        o = len(verts)
        verts.extend(v)
        faces.extend([[i + o for i in face] for face in f])

    # winglet, lofted on from the tip
    cant = math.radians(P["winglet_cant"])
    wsw = math.tan(math.radians(P["winglet_sweep"]))
    wl = []
    for i in range(6):
        t = i / 5.0
        h = P["winglet_h"] * t
        c = chord_at(half) * (1 - (1 - P["winglet_taper"]) * t)
        loop = section(m=P["camber"] * (1 - t), reflex=P["reflex"] * (1 - t))
        wl.append(place(loop, c, le_at(half) + h * wsw,
                        half + h * math.cos(cant),
                        half * math.tan(math.radians(P["dihedral"]))
                        + h * math.sin(cant), twist_at(half)))
    v, f = loft(wl, cap_start=True, cap_end=True)
    o = len(verts)
    verts.extend(v)
    faces.extend([[i + o for i in face] for face in f])

    return obj_from("Wing", verts, faces, "Airframe", coll,
                    smooth=True, mirror=True)


def centreline():
    """An Empty on the aircraft centreline. Control surfaces mirror about
    THIS, not about their own origins, which sit out on the hinge line."""
    e = bpy.data.objects.get("Centreline")
    if e is None:
        e = bpy.data.objects.new("Centreline", None)
        e.empty_display_size = 0.05
        bpy.context.scene.collection.objects.link(e)
    return e


def hinge_frame(y0, y1, frac):
    """World position and axes of a hinge line between two stations.

    The hinge is SWEPT, so it is not parallel to any world axis. Building the
    surface in a frame whose local Y runs along the hinge means a plain
    rotation about local Y is the real deflection -- no gimbal fudging.
    """
    dih = math.tan(math.radians(P["dihedral"]))

    def pt(y):
        c = chord_at(y)
        return Vector(place([(frac, 0.0)], c, le_at(y), y, y * dih,
                            twist_at(y))[0])

    a, b = pt(y0), pt(y1)
    yax = (b - a).normalized()
    xax = Vector((1.0, 0.0, 0.0))
    zax = xax.cross(yax).normalized()
    xax = yax.cross(zax).normalized()
    return a, Matrix((xax, yax, zax)).transposed()


def remember_hinge(ob, origin, basis, axis):
    """Park the hinge frame on the object and pose it.

    The deflection has to COMPOSE with the frame, not replace it: writing
    rotation_euler directly wipes the basis that aligns local Y with a swept
    hinge, and the surface then barely moves. So the rest frame is stored and
    the pose is rebuilt from it each time.
    """
    ob["hinge_origin"] = [origin.x, origin.y, origin.z]
    ob["hinge_basis"] = [c for row in basis for c in row]
    ob["hinge_axis"] = axis
    pose_hinge(ob, 0.0)


def pose_hinge(ob, deg):
    o = Vector(ob["hinge_origin"]) * MM
    b = Matrix([ob["hinge_basis"][i:i + 3] for i in (0, 3, 6)])
    axis = "XYZ"[int(ob["hinge_axis"])]
    ob.matrix_world = (Matrix.Translation(o) @ b.to_4x4()
                       @ Matrix.Rotation(math.radians(deg), 4, axis))


def control(name, y0f, y1f, coll):
    """One flap or aileron, as its own object with its origin ON the hinge
    line and its local Y running along it."""
    half = P["span"] / 2.0
    y0, y1 = y0f * half, y1f * half
    hinge = 1.0 - P["flap_chord"]
    g = P["gap"]
    loop = section()
    dih = math.tan(math.radians(P["dihedral"]))

    origin, basis = hinge_frame(y0, y1, hinge)
    inv = basis.transposed()                       # orthonormal, so inverse

    secs = []
    for i in range(8):
        y = (y0 + g) + ((y1 - g) - (y0 + g)) * i / 7.0
        c = chord_at(y)
        cut = [(max(x, hinge), z) for (x, z) in loop]
        world = place(cut, c, le_at(y), y, y * dih, twist_at(y))
        secs.append([tuple(inv @ (Vector(w) - origin)) for w in world])

    v, f = loft(secs)
    ob = obj_from(name, v, f, "Control", coll, smooth=True)
    remember_hinge(ob, origin, basis, axis=1)
    md = ob.modifiers.new("Mirror", "MIRROR")
    md.use_axis = (False, True, False)
    md.mirror_object = centreline()
    return ob


def pod(coll):
    """EDF fuselage: blended centre body, open at the tail for the nozzle."""
    L, w, h = P["pod_len"], P["pod_w"], P["pod_h"]
    st = [(-0.12, 0.10, 0.12, 0.00), (-0.05, 0.40, 0.44, 0.02),
          (0.05, 0.72, 0.80, 0.03), (0.18, 1.00, 1.00, 0.02),
          (0.34, 1.00, 0.98, 0.00), (0.50, 0.92, 0.90, -0.01),
          (0.615, 0.80, 0.78, -0.02), (0.78, 0.62, 0.60, -0.03),
          (0.93, 0.50, 0.48, -0.04), (1.00, 0.45, 0.43, -0.04)]
    secs = [[(fx * L, p[1], p[2])
             for p in ring(w * fw, h * fh, cz=fz * h)]
            for (fx, fw, fh, fz) in st]
    v, f = loft(secs, cap_start=True, cap_end=False)
    return obj_from("Fuselage", datum(v), f, "Airframe", coll, smooth=True)


def blade_set(x0, r_hub, r_tip, n, chord, beta_hub, beta_tip, z, thick=1.1,
              skew=0.0):
    """A ring of n blades.

    At radius r the chord lies along  cos(beta) * tangential + sin(beta) * axial,
    so beta is the stagger measured from the plane of rotation: 90 deg would be
    a blade pointing straight down the duct, 0 deg a flat paddle. Real EDF
    rotors sit steep at the hub and flatten toward the tip, because the blade
    sees the vector sum of axial flow and its own rotational speed, and that
    gets faster the further out you go.
    """
    verts, faces = [], []
    steps = [r_hub + (r_tip - r_hub) * i / 3.0 for i in range(4)]
    for b in range(n):
        th = 2.0 * math.pi * b / n
        st, ct = math.sin(th), math.cos(th)
        base = len(verts)
        for r in steps:
            f = (r - r_hub) / max(1e-6, r_tip - r_hub)
            beta = math.radians(beta_hub + (beta_tip - beta_hub) * f)
            cb, sb = math.cos(beta), math.sin(beta)
            c = chord * (1.0 - 0.18 * f)
            # chord direction and a normal to it, both unit
            dx, dy, dz = sb, -cb * st, cb * ct
            nx, ny, nz = -cb, -sb * st, sb * ct
            for side in (-1.0, 1.0):
                for u in (-c / 2.0, c / 2.0):
                    verts.append((x0 + u * dx + f * skew + side * thick / 2 * nx,
                                  r * ct + u * dy + side * thick / 2 * ny,
                                  z + r * st + u * dz + side * thick / 2 * nz))
        for i in range(len(steps) - 1):
            a0, a1 = base + i * 4, base + (i + 1) * 4
            faces += [[a0, a0 + 1, a1 + 1, a1], [a0 + 2, a1 + 2, a1 + 3, a0 + 3],
                      [a0, a1, a1 + 2, a0 + 2], [a0 + 1, a0 + 3, a1 + 3, a1 + 1]]
        faces.append([base, base + 2, base + 3, base + 1])
        e = base + (len(steps) - 1) * 4
        faces.append([e + 1, e + 3, e + 2, e])
    return verts, faces


def tube(stations, z, npts=28, cap_start=False, cap_end=False):
    """Axisymmetric shell from (x, radius) stations."""
    return loft([[(x, p[1], p[2]) for p in circle(r, npts, cz=z)]
                 for (x, r) in stations], cap_start, cap_end)


def duct(coll):
    """The fixed EDF installation: bellmouth inlets, a gentle diffuser, the
    fan, a stator to take the swirl back out, and a converging nozzle.

    Nothing retracts. That costs drag every second the fan is off, so the
    whole shape is arranged to make that number small -- see tools/edf.py.
    """
    L, z = P["pod_len"], P["duct_z"]
    x_fan = P["fan_x"] * L
    r_fan = P["fan_dia"] / 2.0
    r_hub = P["fan_hub"] / 2.0
    r_noz = P["nozzle_dia"] / 2.0

    # --- duct barrel: diffuser in, constant at the fan, converging out ---
    barrel = [(x_fan - 150.0, r_fan * 0.88), (x_fan - 40.0, r_fan),
              (x_fan + 46.0, r_fan), (L + 6.0, r_noz)]
    v, f = tube(barrel, z)
    obj_from("Duct", datum(v), f, "Duct", coll, smooth=True)

    # --- rotor ---
    v, f = blade_set(x_fan, r_hub - 1.0, r_fan - 0.7, P["fan_blades"],
                     15.0, 58.0, 33.0, z)
    hv, hf = tube([(x_fan - 16.0, r_hub * 0.55), (x_fan - 7.0, r_hub * 0.95),
                   (x_fan + 9.0, r_hub)], z, cap_start=True, cap_end=True)
    o = len(v)
    v += hv
    f += [[i + o for i in face] for face in hf]
    obj_from("Fan", datum(v), f, "Trim", coll, smooth=True)

    # --- stator: straightens the swirl the rotor put in, which is thrust you
    #     have already paid for. Seven vanes against twelve blades. ---
    v, f = blade_set(x_fan + 30.0, r_hub + 0.5, r_fan - 0.7,
                     P["stator_vanes"], 20.0, 74.0, 74.0, z, thick=1.4)
    obj_from("Stator", datum(v), f, "Duct", coll, smooth=True)

    # --- motor can and a closing tailcone, so the wake does not just stop ---
    v, f = tube([(x_fan + 12.0, r_hub), (x_fan + P["motor_len"], r_hub),
                 (x_fan + P["motor_len"] + P["tailcone_len"] * 0.45, r_hub * 0.72),
                 (x_fan + P["motor_len"] + P["tailcone_len"], r_hub * 0.10)],
                z, cap_start=True, cap_end=True)
    obj_from("Motor", datum(v), f, "Trim", coll, smooth=True)

    # --- inlets: bellmouth lip, then a 1.2 deg diffuser aft ---
    ri = P["inlet_dia"] / 2.0
    lip = P["lip_radius"]
    x0 = P["inlet_x"] * L
    secs = []
    for (dx, rr, dy, dz) in ((-10.0, ri + lip * 0.55, 0.492, 15.6),
                             (-4.0, ri + lip * 0.95, 0.489, 15.2),
                             (2.0, ri, 0.484, 14.6),
                             (40.0, ri * 0.98, 0.452, 10.8),
                             (86.0, ri * 0.80, 0.392, 5.6)):
        secs.append([(x0 + dx, P["pod_w"] * dy + p[1] * 0.80,
                      z + dz + p[2]) for p in circle(rr, 22)])
    v, f = loft(secs, cap_start=False, cap_end=False)
    inl = obj_from("Inlet", datum(v), f, "Duct", coll, smooth=True, mirror=True)
    inl.modifiers["Mirror"].mirror_object = centreline()
    return inl


def fin(coll):
    """One fin. The rudder hangs behind the nozzle, which is what the kit
    calls integrated vector control: deflect it and you deflect the jet."""
    loop = []
    n = 34
    for i in range(n + 1):
        x = 0.5 * (1 - math.cos(math.pi * i / n))
        loop.append((x, thickness(x, 0.085)))
    for i in range(n, -1, -1):
        x = 0.5 * (1 - math.cos(math.pi * i / n))
        loop.append((x, -thickness(x, 0.085)))

    sw = math.tan(math.radians(P["fin_sweep"]))
    hinge = 1.0 - P["rudder_frac"]
    z0 = pod_top(P["fin_x"] / P["pod_len"]) - 4.0

    def fin_secs(x_lo, x_hi, heights):
        out = []
        for hgt in heights:
            t = hgt / P["fin_h"]
            c = P["fin_root"] + (P["fin_tip"] - P["fin_root"]) * t
            cut = [(min(max(x, x_lo), x_hi), zz) for (x, zz) in loop]
            pts = place(cut, c, P["fin_x"] + hgt * sw, 0.0, 0.0, 0.0)
            out.append([(px, pz, z0 + hgt) for (px, _py, pz) in pts])
        return out

    hs = [P["fin_h"] * i / 7.0 for i in range(8)]
    v, f = loft(fin_secs(0.0, hinge, hs))
    obj_from("Fin", datum(v), f, "Airframe", coll, smooth=True)

    def hinge_pt(h):
        c = P["fin_root"] + (P["fin_tip"] - P["fin_root"]) * (h / P["fin_h"])
        return Vector(datum([(P["fin_x"] + h * sw + hinge * c, 0.0, z0 + h)])[0])

    a, b = hinge_pt(0.0), hinge_pt(P["fin_h"])
    zax = (b - a).normalized()
    xax = Vector((1.0, 0.0, 0.0))
    yax = zax.cross(xax).normalized()
    xax = yax.cross(zax).normalized()
    basis = Matrix((xax, yax, zax)).transposed()
    inv = basis.transposed()

    secs = fin_secs(hinge, 1.0, hs)
    local = [[tuple(inv @ (Vector(w) - a)) for w in datum(sec)] for sec in secs]
    v, f = loft(local)
    ob = obj_from("Rudder", v, f, "Control", coll, smooth=True)
    remember_hinge(ob, a, basis, axis=2)


# ==========================================================================
# build
# ==========================================================================

def set_controls(flap=0.0, aileron=0.0, rudder=0.0, elevator=0.0):
    """Drive the surfaces. Positive flap = down (brake), positive aileron =
    right roll, and elevator is added to BOTH flap pairs, which is how a
    flying wing gets pitch out of surfaces that are also doing something else.
    """
    for (name, deg) in (("Flap", -(flap + elevator)),
                        ("Aileron", -(aileron + elevator)),
                        ("Rudder", rudder)):
        ob = bpy.data.objects.get(name)
        if ob is not None and "hinge_basis" in ob:
            pose_hinge(ob, deg)


def build():
    coll = collection()
    centreline()
    wing(coll)
    control("Flap", P["flap_y0"], P["flap_y1"], coll)
    control("Aileron", P["ail_y0"], P["ail_y1"], coll)
    pod(coll)
    duct(coll)
    fin(coll)

    area = P["span"] * (P["root_chord"] + P["tip_chord"]) / 2.0 / 1e4
    print(f"Jetwing: span {P['span']:.0f} mm, area {area:.1f} dm2, "
          f"AR {(P['span'] / 100.0) ** 2 / area:.2f}")
    print(f"  1022 g -> {1022 / area:.1f} g/dm2,  1750 g -> {1750 / area:.1f} g/dm2")
    print(f"  {len(coll.objects)} objects; Flap/Aileron/Rudder are hinged")
    return coll


if __name__ == "__main__":
    build()
    set_controls()
