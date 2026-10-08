#!/usr/bin/env python3
"""
Spar sizing, and where the carbon has to step down.

    python3 tools/spar.py

The printed shell carries torsion and skin loads. The carbon carries the
BENDING, which is what breaks wings, so it is sized from the real span
loading rather than from a rule of thumb:

    shear    S(y) = integral of lift outboard of y
    moment   M(y) = integral of S(y) outboard of y
    needed   Z(y) = M(y) / sigma_allow          (section modulus)
    have     Z    = pi (D^4 - d^4) / (32 D)     (round tube)

The lift distribution comes from the lifting-line solution in optimise.py,
so it is the actual loading of THIS planform, not an assumed ellipse.

The constraint that bites is not strength. It is that the wing gets thin
outboard: at a 106 mm tip chord and 10.5% thickness there is 11.1 mm of
section to put a rod in, and a 10 mm rod leaves no wall. So the spar steps
down, and this works out where.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimise as OPT

G = 9.80665

DESIGN = {
    "n_limit": 6.0,          # limit load factor
    "safety": 1.5,           # ultimate = limit x this
    "sigma": 450.0,          # N/mm2 allowable in a bonded carbon spar
    "t_c": 0.105,            # section thickness ratio
    "t_at_spar": 0.99,       # spar sits at 30% chord, near maximum thickness
    "wall_min": 1.6,         # printed wall each side of the rod
    "fit": 0.25,             # bore clearance on diameter, for a sliding fit
    "spar_frac": 0.30,       # chordwise position
}

# What you can actually buy, OD x ID in mm. Rods have ID 0.
STOCK = [(4.0, 0.0), (5.0, 3.0), (6.0, 4.0), (6.0, 0.0), (8.0, 6.0),
         (8.0, 0.0), (10.0, 8.0), (10.0, 6.0), (12.0, 10.0), (12.0, 8.0)]


def Z_tube(od, idm):
    return math.pi * (od ** 4 - idm ** 4) / (32.0 * od)


def mass_per_m(od, idm, rho=1.55e-3):
    return math.pi / 4.0 * (od ** 2 - idm ** 2) * 1000.0 * rho   # g/m


def span_loads(span, c_root, taper, twist, mass_g, n_lim, n=120):
    """Lift per unit span, then shear and moment by integrating inboard."""
    su = OPT.ll_setup(span, c_root, taper, twist)
    # trim to the design CL is irrelevant for SHAPE; take the loading shape at
    # a representative angle and scale it to carry the design load
    r = OPT.ll_at(su, 6.0)
    local = OPT.ll_local(su, r["coef"])       # (fraction of semi-span, cl)
    half = span / 2.0
    W = mass_g / 1000.0 * G * n_lim
    # lift per unit span proportional to cl(y) * chord(y)
    raw = []
    for (f, cl) in local:
        c = c_root * (1.0 - (1.0 - taper) * f)
        raw.append((f * half, max(0.0, cl) * c))
    tot = 0.0
    for i in range(len(raw) - 1):
        tot += 0.5 * (raw[i][1] + raw[i + 1][1]) * (raw[i + 1][0] - raw[i][0])
    k = (W / 2.0) / tot                       # scale so one wing carries W/2
    stations = [raw[i][0] for i in range(len(raw))]
    w = [raw[i][1] * k for i in range(len(raw))]        # N per mm

    shear, moment = [0.0] * len(w), [0.0] * len(w)
    for i in range(len(w) - 2, -1, -1):
        dy = stations[i + 1] - stations[i]
        shear[i] = shear[i + 1] + 0.5 * (w[i] + w[i + 1]) * dy
    for i in range(len(w) - 2, -1, -1):
        dy = stations[i + 1] - stations[i]
        moment[i] = moment[i + 1] + 0.5 * (shear[i] + shear[i + 1]) * dy
    return stations, w, shear, moment


def available(y, span, c_root, taper, d=DESIGN):
    """Rod diameter the wing section can actually swallow at a station."""
    f = y / (span / 2.0)
    c = c_root * (1.0 - (1.0 - taper) * f)
    t = c * d["t_c"] * d["t_at_spar"]
    return t - 2.0 * d["wall_min"], c, t


def pick(span, c_root, taper, twist, mass_g, d=DESIGN):
    """Walk out the span, choosing the lightest stock that is both strong
    enough and small enough to fit."""
    st, w, shear, mom = span_loads(span, c_root, taper, twist, mass_g,
                                   d["n_limit"] * d["safety"])
    rows = []
    for i, y in enumerate(st):
        need = mom[i] / d["sigma"]                    # mm3
        fits, c, t = available(y, span, c_root, taper, d)
        ok = [(od, idm) for (od, idm) in STOCK
              if Z_tube(od, idm) >= need and od <= fits]
        best = min(ok, key=lambda p: mass_per_m(*p)) if ok else None
        rows.append({"y": y, "M": mom[i], "need": need, "fits": fits,
                     "chord": c, "thick": t, "pick": best})
    return rows


def plan(span, c_root, taper, twist, mass_g, d=DESIGN):
    """Turn the station-by-station picks into a small number of real parts."""
    rows = pick(span, c_root, taper, twist, mass_g, d)
    segs, cur = [], None
    for r in rows:
        p = r["pick"]
        if p is None:
            segs.append({"from": r["y"], "to": r["y"], "tube": None})
            cur = None
            continue
        if cur is None or p != cur["tube"]:
            cur = {"from": r["y"], "to": r["y"], "tube": p}
            segs.append(cur)
        else:
            cur["to"] = r["y"]
    # merge: keep the strongest tube running from the root, step down once
    strong = max((s for s in segs if s["tube"]), key=lambda s: s["tube"][0])
    return rows, segs, strong
