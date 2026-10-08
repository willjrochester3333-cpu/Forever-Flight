#!/usr/bin/env python3
"""
Optimising the Jetwing for glide performance, and pinning the dimensions.

    python3 tools/optimise.py

WHAT IS BEING MAXIMISED

Best lift/drag ratio, with minimum sink reported alongside, because those are
the two numbers that decide how long a wing stays up. Thrust does not appear:
a 50 mm fan on this aircraft is 22% efficient (tools/edf.py), so anything the
motor does is expensive and anything the wing does is free.

HOW, AND WHY NOT A SIMPLER MODEL

Induced drag is most of the drag of a high-aspect-ratio wing at soaring
speeds, and it depends on HOW THE LIFT IS SPREAD ACROSS THE SPAN, which in
turn depends on taper and washout. A CL^2/(pi.AR.e) formula with a guessed e
cannot see that, so it cannot optimise it -- it would just say "more span".

So the span loading is solved properly, by lifting-line theory:

    y = -(b/2) cos(th),      Gamma(th) = 2bV sum A_n sin(n th)

    sum A_n sin(n th) [ n.mu + sin th ] = mu sin th (alpha - alpha_L0 + twist)

    with mu = c(th) a0 / (4b)

    CL = pi.AR.A1          CDi = pi.AR. sum n.A_n^2          e = A1^2 / sum n.A_n^2

That falls straight out of the Fourier coefficients, so e is an OUTPUT of the
planform rather than an assumption about it. The same coefficients give the
local lift coefficient at every station, which is what the tip-stall
constraint needs.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import edf as EDF

RHO = 1.225
NU = 1.46e-5
G = 9.80665

# What is fixed, and why.
FIXED = {
    "span": 2060.0,         # the published BIG WING; not ours to change
    "root_max": 260.0,      # modular joint with the standard wing
    "sweep_le": 24.0,       # the Jetwing shape
    "cl_max": 0.95,         # section maximum, low-Re reflexed aerofoil
    "load_min": 28.0,       # published wing-loading envelope, g/dm2
    "load_max": 48.0,
    "tip_margin": 0.92,     # local cl at the tip, as a fraction of the root's
    "re_tip_min": 55000.0,  # below this a low-Re section falls apart
    "sm": 0.075,            # static margin
}


# ==========================================================================
# lifting line
# ==========================================================================

def lifting_line(b, c_root, taper, twist_tip, a0=2.0 * math.pi, n_terms=24,
                 alpha=5.0, alpha_L0=0.0, twist_power=1.3):
    """Fourier solution of the span loading.

    Returns CL, induced drag, span efficiency and the local lift coefficients.
    Only odd terms are kept: the loading is symmetric, so the even ones are
    zero by construction and including them only makes the matrix singular.
    """
    m = n_terms
    ns = [2 * i + 1 for i in range(m)]            # 1, 3, 5, ...
    thetas = [math.pi * (i + 1) / (2 * m + 1) for i in range(m)]
    S = b * c_root * (1 + taper) / 2.0 / 2.0      # full-span area, mm2
    S *= 2.0 / 2.0
    S = b * c_root * (1 + taper) / 2.0

    A = [[0.0] * m for _ in range(m)]
    rhs = [0.0] * m
    for i, th in enumerate(thetas):
        y = -(b / 2.0) * math.cos(th)
        f = abs(y) / (b / 2.0)
        c = c_root * (1.0 - (1.0 - taper) * f)
        mu = c * a0 / (4.0 * b)
        tw = twist_tip * f ** twist_power
        rhs[i] = mu * math.sin(th) * math.radians(alpha + tw - alpha_L0)
        for j, n in enumerate(ns):
            A[i][j] = math.sin(n * th) * (n * mu + math.sin(th))

    coef = _solve(A, rhs)
    a1 = coef[0]
    AR = b * b / S
    CL = math.pi * AR * a1
    sum_n = sum(n * coef[j] ** 2 for j, n in enumerate(ns))
    CDi = math.pi * AR * sum_n
    e = (a1 * a1 / sum_n) if sum_n > 0 else 0.0

    # local lift coefficient, cl = 4b/c * sum A_n sin(n th)
    local = []
    for th in [math.pi * (i + 1) / 40.0 for i in range(39)]:
        y = -(b / 2.0) * math.cos(th)
        f = abs(y) / (b / 2.0)
        c = c_root * (1.0 - (1.0 - taper) * f)
        g = sum(coef[j] * math.sin(n * th) for j, n in enumerate(ns))
        local.append((f, 4.0 * b * g / c))
    return {"CL": CL, "CDi": CDi, "e": e, "AR": AR, "S": S, "local": local,
            "A": coef}


def _solve(A, b):
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        p = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[p][col]) < 1e-14:
            continue
        M[col], M[p] = M[p], M[col]
        pv = M[col][col]
        for r in range(n):
            if r == col:
                continue
            fac = M[r][col] / pv
            if fac:
                for k in range(col, n + 1):
                    M[r][k] -= fac * M[col][k]
    return [M[i][n] / M[i][i] if abs(M[i][i]) > 1e-14 else 0.0 for i in range(n)]


# ==========================================================================
# drag
# ==========================================================================

def cf(Re, lam_frac=0.35):
    """Mixed laminar/turbulent skin friction. A printed wing at Re ~150k
    holds laminar flow for a good part of the chord, and pretending it is
    fully turbulent throws away the main advantage of flying slowly."""
    if Re < 1000.0:
        return 0.02
    lam = 1.328 / math.sqrt(Re)
    turb = 0.074 / Re ** 0.2
    return lam_frac * lam + (1.0 - lam_frac) * turb


def mass_g(S_dm2):
    """Fixed equipment plus structure that scales with area."""
    fixed = 109 + 40 + 190 + 12 + 45 + 35          # EDF, ESC, pack, RX, servos, bits
    pod = 180.0
    return fixed + pod + 13.6 * S_dm2


def drag_build(b, c_root, taper, S_mm2, mac, V, t_c=0.105, duct=True):
    """Parasite drag, referred to wing area."""
    S = S_mm2 / 1e6
    items = []
    ff_w = 1.0 + 2.0 * t_c + 60.0 * t_c ** 4
    items.append(("wing", 2.03 * S, cf(V * mac / 1000.0 / NU) * ff_w))
    # pod: body of revolution, fineness ~6
    pl, pd = 0.620, 0.100
    fr = pl / pd
    items.append(("fuselage", math.pi * pd * pl * 0.78,
                  cf(V * pl / NU) * (1.0 + 60.0 / fr ** 3 + 0.0025 * fr)))
    items.append(("fin", 2.0 * 0.0225, cf(V * 0.125 / NU) * (1 + 2 * 0.085)))
    items.append(("winglets", 2.0 * 0.0180, cf(V * 0.090 / NU) * ff_w))
    cd0 = sum(a * c for (_n, a, c) in items) / S
    cd0 *= 1.08                                     # interference
    cold = 0.0
    if duct:
        cold = EDF.cold_drag(V)["freewheel"] / (0.5 * RHO * V * V * S)
    return {"items": items, "cd0": cd0, "cold": cold, "total": cd0 + cold}


def performance(c_root, taper, twist, b=None, alpha_L0=0.0, duct=True):
    """Full polar for one geometry: best glide and minimum sink."""
    b = b or FIXED["span"]
    S_mm2 = b * c_root * (1 + taper) / 2.0
    S_dm2 = S_mm2 / 1e4
    m = mass_g(S_dm2)
    W = m / 1000.0 * G
    AR = b * b / S_mm2
    mac = (2.0 / 3.0) * c_root * (1 + taper + taper * taper) / (1 + taper)

    best = {"ld": 0.0}
    sink_best = {"sink": 1e9}
    for i in range(36):
        alpha = -2.0 + 0.5 * i
        ll = lifting_line(b, c_root, taper, twist, alpha=alpha,
                          alpha_L0=alpha_L0)
        CL = ll["CL"]
        if CL < 0.08 or CL > FIXED["cl_max"]:
            continue
        V = math.sqrt(2 * W / (RHO * (S_mm2 / 1e6) * CL))
        d = drag_build(b, c_root, taper, S_mm2, mac, V, duct=duct)
        CD = d["total"] + ll["CDi"]
        ld = CL / CD
        sink = V * CD / CL
        if ld > best["ld"]:
            best = {"ld": ld, "CL": CL, "V": V, "CD": CD, "cd0": d["total"],
                    "cdi": ll["CDi"], "e": ll["e"], "alpha": alpha,
                    "sink": sink}
        if sink < sink_best["sink"]:
            sink_best = {"sink": sink, "CL": CL, "V": V, "ld": ld}

    # stall margin at the tip, evaluated near CL_max
    ll = lifting_line(b, c_root, taper, twist, alpha=11.0, alpha_L0=alpha_L0)
    loc = ll["local"]
    peak = max(cl for (_f, cl) in loc)
    tip = max(cl for (f, cl) in loc if f > 0.90)
    peak_at = max(loc, key=lambda p: p[1])[0]
    c_tip = c_root * taper
    re_tip = (best.get("V", 12.0) * c_tip / 1000.0) / NU

    return {"S_mm2": S_mm2, "S_dm2": S_dm2, "AR": AR, "mac": mac, "mass": m,
            "load": m / S_dm2, "c_root": c_root, "c_tip": c_tip,
            "taper": taper, "twist": twist, "best": best, "min_sink": sink_best,
            "tip_ratio": tip / peak, "peak_at": peak_at, "re_tip": re_tip}


def feasible(p):
    why = []
    if p["c_root"] > FIXED["root_max"] + 1e-6:
        why.append("root chord over the modular joint")
    if not (FIXED["load_min"] <= p["load"] <= FIXED["load_max"]):
        why.append(f"wing loading {p['load']:.1f} outside 28-48")
    if p["tip_ratio"] > FIXED["tip_margin"]:
        why.append(f"tip loaded {p['tip_ratio']:.2f} of peak -- tip stall")
    if p["re_tip"] < FIXED["re_tip_min"]:
        why.append(f"tip Re {p['re_tip']:.0f} too low")
    if not p["best"].get("ld"):
        why.append("no trim point")
    return why


def score(p, objective):
    """Bigger is better, whichever objective is in play."""
    if objective == "sink":
        return -p["min_sink"]["sink"]
    return p["best"]["ld"]


def optimise(objective="ld", root_fixed=None):
    """Search taper, root chord and washout. Coarse grid, then refine.

    objective   "ld" for best glide, "sink" for minimum sink. They are not
                the same design: best glide wants span efficiency at a higher
                speed, minimum sink wants area and a low wing loading.
    root_fixed  pin the root chord, e.g. to keep the modular joint with the
                standard wing. The optimiser will happily throw that away for
                a few percent, and it is not the optimiser's call to make.
    """
    best, rows = None, []
    rng = {"taper": [0.26 + 0.03 * i for i in range(12)],
           "root": ([root_fixed] if root_fixed else
                    [205.0 + 6.0 * i for i in range(10)]),
           "twist": [-0.5 - 0.35 * i for i in range(10)]}
    for _ in range(3):
        best = None
        for tp in rng["taper"]:
            for cr in rng["root"]:
                for tw in rng["twist"]:
                    p = performance(cr, tp, tw)
                    if feasible(p):
                        continue
                    if best is None or score(p, objective) > score(best, objective):
                        best = p
        if best is None:
            return None, rows
        rows.append(dict(best))
        # refine around the winner
        def around(v, step, n=7):
            return [v + step * (i - n // 2) for i in range(n)]
        rng = {"taper": around(best["taper"], 0.008),
               "root": ([root_fixed] if root_fixed
                        else around(best["c_root"], 2.0)),
               "twist": around(best["twist"], 0.08)}
    return best, rows
