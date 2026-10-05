#!/usr/bin/env python3
"""
Airframe variants of the FF-1, exported as 3D files.

    python3 tools/variants.py

Builds the aircraft as the printed plans actually produce it -- no retractable
motor pylon, no retractable turbine mast, and therefore no dorsal spine and no
keel fairing, because those two exist only to swallow the retracting pods.

Four variants:

    glider-2000   2.0 m span, no motor               pure glider
    motor-2000    2.0 m span, nose folding prop      self-launching
    glider-2500   2.5 m span, no motor
    motor-2500    2.5 m span, nose folding prop

The 2500 variants are the whole aircraft scaled by 1.25, not a longer wing on
the same fuselage. Geometric scaling keeps the tail volume coefficients and the
CG position (as a fraction of mean chord) exactly where they are, so the
aircraft stays trimmed and stable. Stretching only the span would cut the fin
volume by a third and leave it directionally soft -- see the table this prints.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analysis as AN
import build_model as B
import config as C
import mesh as M
import plans as PL

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "models", "variants")
IMG = os.path.join(ROOT, "docs", "img", "variants")

VARIANTS = [
    ("glider-2000", 1.00, "none", "2.0 m glider, no motor"),
    ("motor-2000",  1.00, "nose", "2.0 m, nose folding prop"),
    ("glider-2500", 1.25, "none", "2.5 m glider, no motor"),
    ("motor-2500",  1.25, "nose", "2.5 m, nose folding prop"),
]

# How each mass line scales with a geometric factor k.
#   3 = solid structure (volume)   2 = skins and films (area)   0 = bought gear
MASS_EXP = {
    "Wing cores, XPS 30 kg/m3": 3,
    "Wing skin: 25 g/m2 glass or laminating film": 2,
    "Wing spar, 8 mm carbon tube + joiner": 3,
    "Control surfaces, hinges, horns": 2,
    "Fuselage pod, 3 mm depron + ply doublers": 2,
    "Tail boom, 16-12 mm carbon, 700 mm": 2,
    "V-tail panels + mount": 2,
    "Motor, ESC, folding prop (nose)": 0,
    "Battery, 3S 2200 mAh LiPo": 0,
    "RX, 4 servos, wiring": 0,
    "Flight controller, GNSS, airspeed": 0,
}


def variant_mass(scale, motor):
    """Grams, and the itemised breakdown."""
    rows, total = [], 0.0
    for name, g in PL.PROTO_MASS:
        if motor == "none" and name.startswith("Motor,"):
            continue
        gs = g * scale ** MASS_EXP.get(name, 2)
        rows.append((name, gs))
        total += gs
    return total, rows


def variant_cd0(motor):
    """C_D0 built up from the design components, corrected for this airframe."""
    keep = []
    for (n, d, note) in C.DRAG["components"]:
        if "spine" in n.lower() or "ventral" in n.lower():
            continue                      # deleted with the retracting pods
        if "bay door" in n.lower():
            continue                      # no bays
        keep.append((n, d, note))
    base = C.DRAG["wing_cd_min"] + sum(d for _n, d, _t in keep)
    base -= 0.00120                        # no solar cells, so no cell steps
    base += 0.00850                        # foam build: surface, LE radius, linkages
    if motor == "nose":
        base += 0.00030                    # folded blades against the spinner
    return base * (1.0 + C.DRAG["margin"])


def performance(scale, motor):
    """Key numbers for a variant, from the design model."""
    mass_g, _rows = variant_mass(scale, motor)
    mass = mass_g / 1000.0
    W = mass * C.MISSION["g"]
    cd0 = variant_cd0(motor)
    g = AN.geom()
    S = g["S"] * scale ** 2
    b = g["b"] * scale
    # Geometric scaling means EVERYTHING scales -- tail included. Scaling the
    # wing alone is a span stretch, and it is what collapses the tail volumes.
    saved = (C.DRAG["components"], C.DRAG["margin"], C.WING["span"],
             [list(r) for r in C.WING["stations"]],
             {k: C.VTAIL[k] for k in ("le_x", "root_chord", "tip_chord",
                                      "panel_span")})
    try:
        C.DRAG["components"] = [("variant", cd0 / (1 + C.DRAG["margin"])
                                 - C.DRAG["wing_cd_min"], "")]
        C.DRAG["margin"] = 0.0
        C.WING["span"] = C.WING["span"] * scale
        C.WING["stations"] = [(y * scale, c * scale, x * scale, d, t)
                              for (y, c, x, d, t) in C.WING["stations"]]
        for k in ("le_x", "root_chord", "tip_chord", "panel_span"):
            C.VTAIL[k] = C.VTAIL[k] * scale
        kp = AN.key_points(W)
        pms = AN.practical_min_sink(W)
        st = AN.stability()
        return {"mass_g": mass_g, "cd0": cd0, "S": S, "b": b,
                "wing_loading": W / S, "ld": kp["best_glide"]["LD"],
                "v_bg": kp["best_glide"]["V"], "v_stall": kp["v_stall"],
                "sink_min": pms["sink"], "v_ms": pms["V"],
                "mac": AN.geom()["mac"], "vh": st["V_h"], "vv": st["V_v"],
                "cg_mac": st["cg_target"], "np_mac": st["x_np_mac"]}
    finally:
        (C.DRAG["components"], C.DRAG["margin"], C.WING["span"],
         stations, vt) = saved
        C.WING["stations"] = [tuple(r) for r in stations]
        C.VTAIL.update(vt)


def span_stretch_check():
    """What a span-only stretch would do to the tail volumes. For the notes."""
    saved = [list(r) for r in C.WING["stations"]]
    span0 = C.WING["span"]
    base = AN.stability()
    try:
        k = 1.25
        # add span at the existing taper, leaving fuselage and tail alone
        C.WING["span"] = span0 * k
        C.WING["stations"] = [(y * k, c, x, d, t)
                              for (y, c, x, d, t) in C.WING["stations"]]
        stretched = AN.stability()
        g = AN.geom()
        return {"base_vh": base["V_h"], "base_vv": base["V_v"],
                "str_vh": stretched["V_h"], "str_vv": stretched["V_v"],
                "str_S": g["S"], "str_AR": g["AR"]}
    finally:
        C.WING["span"] = span0
        C.WING["stations"] = [tuple(r) for r in saved]


def build(name, scale, motor, solar=False, turret=True):
    saved = dict(B.OPTS)
    try:
        B.OPTS.update({"motor": motor, "turbine": False, "spine": False,
                       "ventral": False, "solar": solar, "turret": turret})
        a, props, _m, _r = B.assemble(True)
    finally:
        B.OPTS.clear()
        B.OPTS.update(saved)          # never leak variant state to a later caller
    if scale != 1.0:
        a.apply(M.scale(scale))
    return a, props


def render_variant(a, name, scale):
    import render as R
    R.render(a, R.iso(35, -20), w=1200, h=780,
             path=os.path.join(IMG, name + ".png"))
    return os.path.join(IMG, name + ".png")


def render_size_comparison(meshes):
    """Both spans in ONE frame, so the size difference is actually visible.

    Rendered separately each image auto-fits and the two look identical.
    """
    import render as R
    scene = M.Mesh("compare")
    small = meshes["motor-2000"].copy()
    big = meshes["motor-2500"].copy()
    big.apply(M.translate(1750.0, 0.0, 0.0))
    scene.merge(small, group_name="motor_pod")
    scene.merge(big, group_name="wing_stbd")
    path = os.path.join(IMG, "size-comparison.png")
    R.render(scene, lambda p: (p[1], -p[0], p[2]), w=1250, h=980, path=path)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    rows, meshes = [], {}
    for (name, scale, motor, desc) in VARIANTS:
        a, _props = build(name, scale, motor)
        meshes[name] = a
        stl = os.path.join(OUT, name + ".stl")
        obj = os.path.join(OUT, name + ".obj")
        M.write_stl(a, stl, f"FF-1 {name}")
        M.write_obj(a, obj, name)
        p = performance(scale, motor)
        lo, hi = a.bounds()
        render_variant(a, name, scale)
        rows.append((name, desc, p, len(a.f), hi[1] - lo[1], hi[0] - lo[0]))
        print(f"{name:<13s} {len(a.f):>6d} tris  span {hi[1] - lo[1]:.0f} mm  "
              f"length {hi[0] - lo[0]:.0f} mm  "
              f"{os.path.getsize(stl) / 1024:.0f} kB")

    print()
    print(f"{'variant':<13s} {'mass':>7s} {'W/S':>9s} {'stall':>7s} "
          f"{'L/D':>6s} {'min sink':>9s} {'V_h':>6s} {'V_v':>7s} {'CG':>7s}")
    for (name, desc, p, _t, _b, _l) in rows:
        print(f"{name:<13s} {p['mass_g']:>6.0f}g "
              f"{p['wing_loading'] / C.MISSION['g'] * 10:>7.1f} g/dm2 "
              f"{p['v_stall']:>6.1f} {p['ld']:>6.1f} "
              f"{p['sink_min']:>8.2f}  {p['vh']:>5.2f} {p['vv']:>6.3f} "
              f"{p['cg_mac'] * 100:>6.1f}%")

    print("wrote", os.path.relpath(render_size_comparison(meshes), ROOT))
    s = span_stretch_check()
    print()
    print("Why the 2500 is a whole-aircraft scale, not a longer wing:")
    print(f"  geometric scale x1.25   V_h {rows[2][2]['vh']:.3f}  "
          f"V_v {rows[2][2]['vv']:.4f}   (unchanged - stays trimmed)")
    print(f"  span stretch to 2.5 m   V_h {s['str_vh']:.3f}  "
          f"V_v {s['str_vv']:.4f}   on the same fuselage and tail")
    print(f"  targets                 V_h 0.45-0.70     V_v 0.020-0.035")
    return rows


if __name__ == "__main__":
    main()
