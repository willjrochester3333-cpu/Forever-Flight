#!/usr/bin/env python3
"""
FF-1 flight log analysis: graphs, a fitted drag polar, and predictions.

    python3 tools/flight_analysis.py data/example_flight.csv
    python3 tools/flight_analysis.py MYFLIGHT.BIN --mass 1.68 --open

Reads an ArduPilot .BIN / .log or any CSV (see tools/flightlog.py), then:

  1. derives climb rate, density, power and energy
  2. segments the flight into powered / thermalling / gliding / regen
  3. finds stabilised glide legs and FITS THE DRAG POLAR
  4. compares the fit against the design build-up in tools/analysis.py
  5. re-runs the endurance model on the MEASURED polar
  6. writes a self-contained HTML report with SVG charts

The polar fit is the point of the exercise. Sink rate against airspeed is

    w(V) = a·V³ + b/V          a = C_D0·ρS/2W,   b = 2W/(ρS·π·AR·e)

which is linear in (a, b), so two-parameter least squares recovers C_D0 and
Oswald e from a handful of stabilised glides. Everything downstream -- L/D,
minimum sink, endurance -- follows in closed form.

No dependencies. Standard library only.
"""

from __future__ import annotations

import html
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analysis as AN
import config as C
import flightlog as FL

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RHO0 = 1.225
G = C.MISSION["g"]

# Categorical palette, fixed order, validated (see docs/FLIGHT_ANALYSIS.md).
# 1 powered/motor  2 turbine/regen  3 solar  4 glide/measured
PAL_LIGHT = ["#BD4E14", "#0C87AA", "#A16D00", "#7C3B99"]
PAL_DARK = ["#D2622A", "#1D9AC0", "#B98110", "#9A57B5"]

PHASE_ORDER = ["powered", "thermal", "regen", "glide"]
PHASE_SLOT = {"powered": 0, "regen": 1, "thermal": 2, "glide": 3}
PHASE_LABEL = {"powered": "Motor on", "thermal": "Thermalling",
               "regen": "Turbine deployed", "glide": "Gliding",
               "ground": "On the ground"}


# ==========================================================================
# derivation
# ==========================================================================

def isa_density(alt_m, temp_c=None):
    if temp_c is not None:
        T = temp_c + 273.15
        p = 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588
        return p / (287.058 * T)
    T = 288.15 - 0.0065 * alt_m
    return 1.225 * (T / 288.15) ** 4.2561


def slope(ts, vs):
    """Least-squares slope of vs against ts. None if degenerate."""
    n = len(ts)
    if n < 3:
        return None
    mt = sum(ts) / n
    mv = sum(vs) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    if sxx <= 0:
        return None
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / sxx


def derive(tab, mass_kg):
    """Add climb, density, and electrical power/energy columns in place."""
    t = tab["t"]
    dt = tab["dt"]
    n = len(t)
    alt = tab.get("alt")

    # climb rate: least-squares slope over a centred window, so baro noise is
    # averaged rather than differentiated
    half = max(2, int(2.0 / dt))
    climb = [None] * n
    if alt:
        for i in range(n):
            a, b = max(0, i - half), min(n, i + half + 1)
            ts = [t[k] for k in range(a, b) if alt[k] is not None]
            vs = [alt[k] for k in range(a, b) if alt[k] is not None]
            climb[i] = slope(ts, vs)
    if "climb" in tab:                       # prefer a logged vario if present
        tab["climb_derived"] = climb
        tab["climb"] = [c if c is not None else d
                        for c, d in zip(tab["climb"], climb)]
    else:
        tab["climb"] = climb

    temp = tab.get("temp")
    tab["rho"] = [isa_density(alt[i] if alt and alt[i] is not None else 0.0,
                              temp[i] if temp and temp[i] is not None else None)
                  for i in range(n)]

    volt, curr = tab.get("volt"), tab.get("curr")
    if volt and curr:
        tab["p_batt"] = [(volt[i] * curr[i]) if (volt[i] is not None
                                                 and curr[i] is not None) else None
                         for i in range(n)]
    for key in ("solar_w", "turbine_w", "p_batt"):
        if key in tab:
            acc, out = 0.0, []
            for v in tab[key]:
                if v is not None:
                    acc += v * dt / 3600.0
                out.append(acc)
            tab["wh_" + key] = out
    tab["mass"] = mass_kg
    return tab


# ==========================================================================
# segmentation
# ==========================================================================

def classify(tab):
    t, n = tab["t"], len(tab["t"])
    thr = tab.get("throttle")
    roll = tab.get("roll")
    mast = tab.get("mast_mm")
    turb = tab.get("turbine_w")
    asp = tab.get("airspeed") or tab.get("groundspeed")
    alt = tab.get("alt")

    raw = []
    for i in range(n):
        v = asp[i] if asp and asp[i] is not None else 0.0
        a = alt[i] if alt and alt[i] is not None else 0.0
        if v < 3.0 and a < 20.0:
            raw.append("ground")
        elif thr and thr[i] is not None and thr[i] > 0.05:
            raw.append("powered")
        elif (mast and mast[i] is not None and mast[i] > 20.0) or \
             (turb and turb[i] is not None and turb[i] > 1.0):
            raw.append("regen")
        elif roll and roll[i] is not None and abs(roll[i]) > 15.0:
            raw.append("thermal")
        else:
            raw.append("glide")

    # median filter: a single banked sample is not a thermal
    w = max(1, int(6.0 / tab["dt"]))
    sm = []
    for i in range(n):
        win = raw[max(0, i - w):i + w + 1]
        sm.append(max(set(win), key=win.count))
    tab["phase"] = sm

    segs, start = [], 0
    for i in range(1, n + 1):
        if i == n or sm[i] != sm[start]:
            segs.append({"mode": sm[start], "i0": start, "i1": i - 1,
                         "t0": t[start], "t1": t[i - 1],
                         "dur": t[i - 1] - t[start]})
            start = i
    tab["segments"] = [s for s in segs if s["dur"] >= 2.0]
    return tab


def phase_totals(tab):
    out = {}
    for s in tab["segments"]:
        out[s["mode"]] = out.get(s["mode"], 0.0) + s["dur"]
    return out


# ==========================================================================
# polar fit
# ==========================================================================

def glide_windows(tab, win_s=20.0, max_bank=6.0, max_accel=0.05,
                  max_spread=0.6):
    """Stabilised, wings-level, unpowered legs suitable for a polar point."""
    t, dt = tab["t"], tab["dt"]
    asp = tab.get("airspeed")
    used_gs = False
    if not asp:
        asp = tab.get("groundspeed")
        used_gs = True
    alt, roll, phase = tab.get("alt"), tab.get("roll"), tab["phase"]
    if not asp or not alt:
        return [], used_gs

    step = int(win_s / dt)
    out = []
    i = 0
    n = len(t)
    while i + step <= n:
        sl = slice(i, i + step)
        ph = phase[sl]
        if any(p not in ("glide",) for p in ph):
            i += step // 2
            continue
        vs = [v for v in asp[sl] if v is not None]
        als = [a for a in alt[sl] if a is not None]
        rl = [abs(r) for r in (roll[sl] if roll else []) if r is not None]
        if len(vs) < step * 0.8 or len(als) < step * 0.8:
            i += step // 2
            continue
        if rl and max(rl) > max_bank:
            i += step // 2
            continue
        if max(vs) - min(vs) > max_spread * 2:
            i += step // 2
            continue
        ts = [t[k] for k in range(i, i + step)]
        dv = slope(ts, asp[sl])
        w = slope(ts, alt[sl])
        if dv is None or w is None or abs(dv) > max_accel:
            i += step // 2
            continue
        vbar = sum(vs) / len(vs)
        rho = sum(tab["rho"][sl]) / step
        out.append({"t": ts[0], "dur": win_s, "v_ias": vbar,
                    "sink": -w, "rho": rho, "n": len(vs),
                    "alt": sum(als) / len(als),
                    # reduce to sea-level equivalent: same IAS, sink scales
                    # with sqrt(rho/rho0)
                    "sink_sl": -w * math.sqrt(rho / RHO0)})
        i += step
    return out, used_gs


def _solve(M, y):
    """Gaussian elimination with partial pivoting on a small dense system."""
    n = len(y)
    A = [row[:] + [y[i]] for i, row in enumerate(M)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(A[r][c]))
        if abs(A[piv][c]) < 1e-30:
            return None
        A[c], A[piv] = A[piv], A[c]
        for r in range(n):
            if r == c:
                continue
            f = A[r][c] / A[c][c]
            for k in range(c, n + 1):
                A[r][k] -= f * A[c][k]
    return [A[i][n] / A[i][i] for i in range(n)]


def fit_polar(points, mass_kg, S, AR, robust=True, terms=None):
    """Fit the sink polar, then convert it to a drag polar.

    The aircraft's drag is not a clean C_D0 + C_L^2/(pi.AR.e): a real section
    has profile drag that rises either side of its design lift coefficient, so

        C_D = A + D.C_L + B.C_L^2

    Substituting C_L = K/V^2 with K = 2W/(rho.S) and w = V.C_D/C_L gives

        w(V) = (A/K).V^3 + D.V + (B.K)/V

    -- three terms, linear in the coefficients. Fitting only V^3 and 1/V (the
    textbook two-term polar) forces D = 0, which biases both C_D0 and e; on
    synthetic data with a known answer it came out 12 % and 20 % low. So the
    default is the three-term fit whenever there are enough well-spread points.
    """
    if len(points) < 3:
        return None
    W = mass_kg * G
    K = 2.0 * W / (RHO0 * S)
    pts = list(points)

    vs_all = [p["v_ias"] for p in pts]
    if terms is None:
        spread = (max(vs_all) - min(vs_all)) / max(1e-9, min(vs_all))
        terms = 3 if (len(pts) >= 6 and spread > 0.35) else 2

    def basis(V):
        return [V ** 3, 1.0 / V] + ([V] if terms == 3 else [])

    # Column scaling. V^3 and 1/V differ by ~4 orders of magnitude, which makes
    # the raw normal equations numerically singular; normalising each column by
    # its RMS fixes the conditioning and is undone afterwards.
    nb = terms
    rms = [math.sqrt(sum(basis(p["v_ias"])[r] ** 2 for p in pts) / len(pts))
           or 1.0 for r in range(nb)]

    coef, sd = None, 0.0
    for _it in range(3 if robust else 1):
        rms = [math.sqrt(sum(basis(p["v_ias"])[r] ** 2 for p in pts) / len(pts))
               or 1.0 for r in range(nb)]
        M = [[0.0] * nb for _ in range(nb)]
        y = [0.0] * nb
        for p in pts:
            bs = [b / rms[r] for r, b in enumerate(basis(p["v_ias"]))]
            wt = p["n"]
            for r in range(nb):
                for c in range(nb):
                    M[r][c] += wt * bs[r] * bs[c]
                y[r] += wt * bs[r] * p["sink_sl"]
        sc = _solve(M, y)
        if sc is None:
            return None
        coef = [sc[r] / rms[r] for r in range(nb)]
        res = [(p, p["sink_sl"] - sum(ci * bi for ci, bi in
                                      zip(coef, basis(p["v_ias"])))) for p in pts]
        rs = [r for _p, r in res]
        dof = max(1, len(rs) - terms)
        sd = math.sqrt(sum(r * r for r in rs) / dof)
        if not robust or len(pts) <= terms + 2:
            break
        # A point well BELOW the curve was flown in rising air, not still air.
        # Drop it and refit. Points above are honest (sink, or turbulence).
        keep = [p for p, r in res if r > -2.0 * sd]
        if len(keep) == len(pts) or len(keep) < terms + 1:
            break
        pts = keep

    a = coef[0]
    b = coef[1]
    d = coef[2] if terms == 3 else 0.0
    if a <= 0 or b <= 0:
        return None

    # drag-polar coefficients, C_D = A + D.C_L + B.C_L^2
    A_c, D_c, B_c = a * K, d, b / K

    def curve(V):
        return a * V ** 3 + b / V + d * V

    def cd_at(CL):
        return A_c + D_c * CL + B_c * CL * CL

    # operating points, found numerically so they hold for either fit form
    best = None
    V = 5.0
    while V <= 30.0:
        w = curve(V)
        if w > 0:
            ld = V / w
            if best is None or ld > best[1]:
                best = (V, ld, w)
        V += 0.01
    v_bg, ld_max, sink_bg = best

    ms = None
    V = 5.0
    while V <= 30.0:
        w = curve(V)
        if w > 0 and (ms is None or w < ms[1]):
            ms = (V, w)
        V += 0.01
    v_ms, sink_min = ms

    # An equivalent C_D0 / e needs the section's profile-drag curvature, which
    # a flight polar cannot separate from induced drag. Use the design value
    # for k and say so.
    k_sec = C.DRAG["wing_cd_k"]
    cl_d = -D_c / (2.0 * k_sec) if k_sec > 0 else None
    cd0_eq = A_c - k_sec * cl_d * cl_d if cl_d is not None else A_c
    binv = B_c - k_sec
    e_eq = 1.0 / (math.pi * AR * binv) if binv > 1e-9 else None

    wbar = sum(p["sink_sl"] for p in pts) / len(pts)
    sstot = sum((p["sink_sl"] - wbar) ** 2 for p in pts)
    ssres = sum((p["sink_sl"] - curve(p["v_ias"])) ** 2 for p in pts)

    return {"terms": terms, "a": a, "b": b, "d": d,
            "A": A_c, "D": D_c, "B": B_c, "K": K,
            "cd0": cd0_eq, "e": e_eq, "cl_design_implied": cl_d,
            "v_bg": v_bg, "ld_max": ld_max, "sink_bg": sink_bg,
            "v_ms": v_ms, "sink_min": sink_min,
            "r2": 1.0 - ssres / sstot if sstot > 0 else 0.0,
            "resid_sd": sd, "n_used": len(pts), "n_total": len(points),
            "mass": mass_kg, "points": pts,
            "rejected": [p for p in points if p not in pts],
            "curve": curve, "cd_at": cd_at}


def _fit_core(pts, mass_kg, S, AR, terms):
    """Bare fit used by CV and bootstrap -- no robust pass, no derived points."""
    W = mass_kg * G
    K = 2.0 * W / (RHO0 * S)

    def basis(V):
        return [V ** 3, 1.0 / V] + ([V] if terms == 3 else [])

    nb = terms
    rms = [math.sqrt(sum(basis(p["v_ias"])[r] ** 2 for p in pts) / len(pts))
           or 1.0 for r in range(nb)]
    M = [[0.0] * nb for _ in range(nb)]
    y = [0.0] * nb
    for p in pts:
        bs = [b / rms[r] for r, b in enumerate(basis(p["v_ias"]))]
        for r in range(nb):
            for c in range(nb):
                M[r][c] += p["n"] * bs[r] * bs[c]
            y[r] += p["n"] * bs[r] * p["sink_sl"]
    sc = _solve(M, y)
    if sc is None:
        return None
    coef = [sc[r] / rms[r] for r in range(nb)]
    a, b = coef[0], coef[1]
    d = coef[2] if terms == 3 else 0.0
    if a <= 0 or b <= 0:
        return None

    def curve(V):
        return a * V ** 3 + b / V + d * V

    best = None
    V = 5.0
    while V <= 30.0:
        w = curve(V)
        if w > 0:
            ld = V / w
            if best is None or ld > best[1]:
                best = (V, ld)
        V += 0.02
    k_sec = C.DRAG["wing_cd_k"]
    A_c, D_c, B_c = a * K, d, b / K
    cl_d = -D_c / (2.0 * k_sec) if k_sec > 0 else 0.0
    binv = B_c - k_sec
    return {"curve": curve, "v_bg": best[0], "ld_max": best[1],
            "cd0": A_c - k_sec * cl_d * cl_d,
            "e": (1.0 / (math.pi * AR * binv)) if binv > 1e-9 else None}


def choose_terms(points, mass_kg, S, AR):
    """Leave-one-out CV on sink prediction. Lower error wins."""
    out = {}
    for terms in (2, 3):
        if len(points) < terms + 2:
            continue
        errs = []
        for i in range(len(points)):
            sub_pts = points[:i] + points[i + 1:]
            f = _fit_core(sub_pts, mass_kg, S, AR, terms)
            if f:
                errs.append((f["curve"](points[i]["v_ias"])
                             - points[i]["sink_sl"]) ** 2)
        if errs:
            out[terms] = math.sqrt(sum(errs) / len(errs))
    if not out:
        return 2, out
    return min(out, key=out.get), out


def bootstrap(points, mass_kg, S, AR, terms, n_boot=300, seed=11):
    """Resample the glide legs with replacement; report percentile spreads.

    This is the honest way to state a fitted C_D0: the sink curve is well
    determined, but splitting it into parasite and induced drag amplifies
    noise, and the spread says by how much.
    """
    import random as _r
    rnd = _r.Random(seed)
    keys = ("cd0", "e", "ld_max", "v_bg")
    draws = {k: [] for k in keys}
    n = len(points)
    for _i in range(n_boot):
        samp = [points[rnd.randrange(n)] for _ in range(n)]
        if len({round(p["v_ias"], 2) for p in samp}) < terms + 1:
            continue
        f = _fit_core(samp, mass_kg, S, AR, terms)
        if not f:
            continue
        for k in keys:
            if f.get(k) is not None:
                draws[k].append(f[k])
    out = {}
    for k, vals in draws.items():
        if len(vals) < 20:
            continue
        vals.sort()
        out[k] = {"lo": vals[int(0.10 * len(vals))],
                  "hi": vals[int(0.90 * len(vals))],
                  "med": vals[len(vals) // 2], "n": len(vals)}
    return out


# ==========================================================================
# predictions
# ==========================================================================

def thermal_stats(tab):
    """Measured climb performance and how much of the flight was in lift."""
    climb = tab.get("climb")
    if not climb:
        return None
    dt = tab["dt"]
    th = [climb[i] for i, p in enumerate(tab["phase"])
          if p == "thermal" and climb[i] is not None]
    airborne = sum(s["dur"] for s in tab["segments"] if s["mode"] != "ground")
    t_lift = sum(dt for c in th if c > 0)
    if not th:
        return {"n": 0, "airborne_s": airborne}
    pos = [c for c in th if c > 0]
    s_circle = None
    return {"n": len(th), "airborne_s": airborne,
            "mean_climb": sum(pos) / len(pos) if pos else 0.0,
            "best_climb": max(th),
            "p90_climb": sorted(pos)[int(0.9 * len(pos))] if pos else 0.0,
            "f_lift": t_lift / airborne if airborne > 0 else 0.0,
            "thermal_frac": sum(dt for c in th) / airborne if airborne else 0.0,
            "hist": th}


def energy_accounting(tab):
    out = {}
    for key, label in (("wh_solar_w", "solar_in"),
                       ("wh_turbine_w", "turbine_in"),
                       ("wh_p_batt", "battery_net")):
        if key in tab and tab[key]:
            out[label] = tab[key][-1]
    if "used_mah" in tab and tab["used_mah"]:
        vals = [v for v in tab["used_mah"] if v is not None]
        if vals:
            nom = C.BATTERY["cells_series"] * 3.6
            out["used_wh_from_mah"] = (vals[-1] - vals[0]) / 1000.0 * nom
    return out


def drag_scale(fit, mass_kg, lo=0.5, hi=2.5):
    """One robust number: the factor on design C_D0 that reproduces measured L/D.

    Splitting a measured sink curve into parasite and induced drag is often not
    identifiable (see the bootstrap intervals), but "how much more drag than
    predicted" always is -- it is a single parameter fitted to a single
    well-determined quantity.
    """
    W = mass_kg * G
    base = AN.cd0_clean()
    saved = (C.DRAG["components"], C.DRAG["margin"])

    def ld_for(scale):
        C.DRAG["components"] = [("scaled", base * scale - C.DRAG["wing_cd_min"], "")]
        C.DRAG["margin"] = 0.0
        return AN.key_points(W)["best_glide"]["LD"]

    try:
        target = fit["ld_max"]
        if ld_for(lo) < target or ld_for(hi) > target:
            pass                                   # target outside the bracket
        for _i in range(60):                       # L/D falls as scale rises
            mid = 0.5 * (lo + hi)
            if ld_for(mid) > target:
                lo = mid
            else:
                hi = mid
        scale = 0.5 * (lo + hi)
        C.DRAG["components"] = [("scaled", base * scale - C.DRAG["wing_cd_min"], "")]
        C.DRAG["margin"] = 0.0
        kp = AN.key_points(W)
        return {"scale": scale, "cd0_eff": base * scale,
                "ld": kp["best_glide"]["LD"], "v_bg": kp["best_glide"]["V"],
                "sink_bg": kp["best_glide"]["sink"]}
    finally:
        C.DRAG["components"], C.DRAG["margin"] = saved


def identifiable(bs, key, must_be_positive=True):
    """Is a bootstrapped coefficient worth quoting?"""
    b = bs.get(key)
    if not b:
        return False, "no interval (too few legs)"
    if must_be_positive and b["lo"] <= 0:
        return False, "interval includes physically impossible values"
    width = (b["hi"] - b["lo"]) / max(abs(b["med"]), 1e-9)
    if width > 0.5:
        return False, f"interval spans {width * 100:.0f} % of the value"
    return True, f"+/-{width * 50:.0f} %"


def predict(fit, tab, thermals, mass_kg, bs):
    """Everything that follows from the measured sink curve."""
    g = AN.geom()
    m = AN.mass_rollup()
    W_design = m["mtow_g"] / 1000.0 * G
    W = mass_kg * G
    kp = AN.key_points(W_design)

    design = {"cd0": AN.cd0_clean(), "e": C.DRAG["oswald_e"],
              "mass": m["mtow_g"] / 1000.0,
              "ld_max": kp["best_glide"]["LD"],
              "v_bg": kp["best_glide"]["V"],
              "sink_bg": kp["best_glide"]["sink"],
              "sink_min": AN.practical_min_sink(W_design)["sink"]}

    # Assumption-free comparison: both curves evaluated at the same speeds.
    sink_cmp = []
    for V in (8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0):
        meas = fit["curve"](V)
        des = AN.glide_sink_at(V, W_design)
        if des and meas > 0:
            sink_cmp.append({"v": V, "measured": meas, "design": des,
                             "delta_pct": 100.0 * (meas / des - 1.0)})

    ds = drag_scale(fit, mass_kg)
    ld_b = bs.get("ld_max")
    scale_lo = scale_hi = None
    if ld_b:
        scale_lo = drag_scale({"ld_max": ld_b["hi"]}, mass_kg)["scale"]
        scale_hi = drag_scale({"ld_max": ld_b["lo"]}, mass_kg)["scale"]

    out = {"design": design, "sink_cmp": sink_cmp, "drag_scale": ds,
           "scale_lo": scale_lo, "scale_hi": scale_hi,
           "identifiable": {k: identifiable(bs, k, k != "e" or True)
                            for k in ("cd0", "e", "ld_max", "v_bg")},
           "delta": {k: 100.0 * (fit[k] / design[k] - 1.0)
                     for k in ("ld_max", "v_bg", "sink_min")
                     if fit.get(k) and design.get(k)}}

    # endurance re-run with the measured drag level
    saved = (C.DRAG["components"], C.DRAG["margin"])
    try:
        C.DRAG["components"] = [
            ("Measured drag level (fitted from flight)",
             ds["cd0_eff"] - C.DRAG["wing_cd_min"],
             f"design C_D0 x {ds['scale']:.3f}")]
        C.DRAG["margin"] = 0.0
        AN._HARVEST_CACHE.clear()
        out["endurance_measured"] = {
            site[0]: AN.simulate(site, launch_h=9.0)["endurance_h"]
            for site in C.MISSION["sites"]}
        if thermals and thermals.get("n"):
            w_air = max(thermals["mean_climb"] + AN.circling_sink(W), 0.2)
            out["harvest_measured"] = AN.soaring_harvest(
                W, w_air, min(0.6, max(0.05, thermals["f_lift"])))
            out["harvest_inputs"] = {"w": w_air, "f": min(0.6, thermals["f_lift"])}
    finally:
        C.DRAG["components"], C.DRAG["margin"] = saved
        AN._HARVEST_CACHE.clear()

    out["endurance_design"] = {
        site[0]: AN.simulate(site, launch_h=9.0)["endurance_h"]
        for site in C.MISSION["sites"]}
    return out


# ==========================================================================
# report
# ==========================================================================

def _dsample(xs, ys, target=900):
    """Downsample a series for SVG, keeping local extremes."""
    n = len(xs)
    if n <= target:
        return [(x, y) for x, y in zip(xs, ys) if y is not None]
    step = n / target
    out = []
    i = 0.0
    while i < n:
        a, b = int(i), min(n, int(i + step))
        chunk = [(xs[k], ys[k]) for k in range(a, b) if ys[k] is not None]
        if chunk:
            out.append(min(chunk, key=lambda p: p[1]))
            hi = max(chunk, key=lambda p: p[1])
            if hi is not out[-1]:
                out.append(hi)
        i += step
    out.sort(key=lambda p: p[0])
    return out


def _nice(lo, hi, n=5):
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            stepv = m * mag
            break
    else:
        stepv = 10 * mag
    start = math.floor(lo / stepv) * stepv
    ticks, v = [], start
    while v <= hi + stepv * 0.5:
        ticks.append(v)
        v += stepv
    return ticks


def chart_time(series, ylabel, title, phases=None, tspan=None, h=250,
               unit="", cid="c"):
    """Multi-series time chart. series: [(label, [(t,y)], slot)]."""
    W, ml, mr, mt, mb = 980, 62, 108, 14, 34
    strip = 14 if phases else 0
    H = h + strip
    pts = [p for _l, s, _k in series for p in s]
    if not pts:
        return ""
    t0 = tspan[0] if tspan else min(p[0] for p in pts)
    t1 = tspan[1] if tspan else max(p[0] for p in pts)
    ylo = min(p[1] for p in pts)
    yhi = max(p[1] for p in pts)
    pad = (yhi - ylo) * 0.08 or 1.0
    # a chart of altitude or airspeed must not invent negative values
    lo_pad = ylo - pad if ylo < 0 else max(0.0, ylo - pad) if ylo > pad else 0.0
    ticks = _nice(lo_pad, yhi + pad)
    ylo, yhi = ticks[0], ticks[-1]

    def px(t):
        return ml + (t - t0) / max(1e-9, t1 - t0) * (W - ml - mr)

    def py(v):
        return mt + (yhi - v) / max(1e-9, yhi - ylo) * (h - mt - mb)

    g = []
    for v in ticks:
        g.append(f'<line x1="{ml}" y1="{py(v):.1f}" x2="{W - mr}" '
                 f'y2="{py(v):.1f}" class="grid"/>')
        g.append(f'<text x="{ml - 8}" y="{py(v) + 3.5:.1f}" text-anchor="end" '
                 f'class="tick">{v:g}</text>')
    for tv in _nice(t0, t1, 6):
        if t0 <= tv <= t1:
            g.append(f'<line x1="{px(tv):.1f}" y1="{mt}" x2="{px(tv):.1f}" '
                     f'y2="{h - mb}" class="grid"/>')
            g.append(f'<text x="{px(tv):.1f}" y="{h - mb + 16}" '
                     f'text-anchor="middle" class="tick">{tv / 60:.0f}</text>')

    body = []
    for (label, s, slot) in series:
        d = " ".join(("M" if i == 0 else "L") + f"{px(t):.1f} {py(v):.1f}"
                     for i, (t, v) in enumerate(s))
        body.append(f'<path d="{d}" fill="none" class="s{slot}" '
                    f'stroke-width="2" stroke-linejoin="round"/>')
        if s:
            body.append(f'<text x="{W - mr + 8}" y="{py(s[-1][1]) + 4:.1f}" '
                        f'class="dlab f{slot}">{html.escape(label)}</text>')

    ph = []
    if phases:
        for (mode, a, b) in phases:
            if mode == "ground":
                continue
            x0, x1 = px(a), px(b)
            ph.append(f'<rect x="{x0:.1f}" y="{h - mb + 22}" '
                      f'width="{max(0.6, x1 - x0 - 0.5):.1f}" height="{strip - 4}" '
                      f'class="p{PHASE_SLOT[mode]}"/>')

    return (f'<figure><figcaption class="ctitle">{html.escape(title)}'
            f'<span class="cunit">{html.escape(unit)}</span></figcaption>'
            f'<div class="cwrap" data-cid="{cid}">'
            f'<svg viewBox="0 0 {W} {H}" class="chart" role="img" '
            f'aria-label="{html.escape(title)}">'
            f'<text x="14" y="{mt + 8}" class="axl" '
            f'transform="rotate(-90 14 {mt + 8})" text-anchor="end">'
            f'{html.escape(ylabel)}</text>'
            + "".join(g) + "".join(body) + "".join(ph)
            + f'<text x="{ml}" y="{H - 2}" class="axl">MINUTES FROM LAUNCH</text>'
            + f'<line class="xhair" x1="0" y1="{mt}" x2="0" y2="{h - mb}" '
              f'style="display:none"/>'
            + '</svg><div class="tip" hidden></div></div></figure>')


def chart_polar(fit, design_curve, cid="polar"):
    """Measured sink polar: legs, fitted curve, design curve. Sink inverted."""
    W, H, ml, mr, mt, mb = 700, 400, 58, 178, 16, 44
    pts = fit["points"] + fit["rejected"]
    vs = [p["v_ias"] for p in pts]
    # start at zero: best glide is where a line from the ORIGIN is tangent to
    # the curve, and that construction only reads if the origin is on the page
    x0, x1 = 0.0, max(vs) + 2.0
    # Scale y from the MEASURED legs (plus a little headroom), not from the
    # fitted curve: the 1/V term diverges as V -> 0 and would squash the
    # interesting region into the top 5 % of the plot.
    ws = [p["sink_sl"] for p in pts] + [fit["curve"](max(vs))]
    y0, y1 = 0.0, max(ws) * 1.35

    def px(v):
        return ml + (v - x0) / (x1 - x0) * (W - ml - mr)

    def py(w):
        return mt + (w - y0) / (y1 - y0) * (H - mt - mb)

    g = []
    for v in _nice(x0, x1, 5):
        if x0 <= v <= x1:
            g.append(f'<line x1="{px(v):.1f}" y1="{mt}" x2="{px(v):.1f}" '
                     f'y2="{H - mb}" class="grid"/>')
            g.append(f'<text x="{px(v):.1f}" y="{H - mb + 16}" '
                     f'text-anchor="middle" class="tick">{v:g}</text>')
    for w in _nice(y0, y1, 5):
        if y0 <= w <= y1:
            g.append(f'<line x1="{ml}" y1="{py(w):.1f}" x2="{W - mr}" '
                     f'y2="{py(w):.1f}" class="grid"/>')
            g.append(f'<text x="{ml - 8}" y="{py(w) + 3.5:.1f}" text-anchor="end" '
                     f'class="tick">{w:.1f}</text>')

    def path(fn, cls, dash=""):
        d, v = [], x0 + 0.4
        while v <= x1:
            y = fn(v)
            if y and 0 < y <= y1:
                d.append(("M" if not d else "L") + f"{px(v):.1f} {py(y):.1f}")
            v += 0.1
        return (f'<path d="{" ".join(d)}" fill="none" class="{cls}" '
                f'stroke-width="2" {dash}/>') if d else ""

    body = [path(lambda v: design_curve(v), "sdes", 'stroke-dasharray="5 4"'),
            path(fit["curve"], "s3")]
    for p in fit["points"]:
        body.append(f'<circle cx="{px(p["v_ias"]):.1f}" cy="{py(p["sink_sl"]):.1f}" '
                    f'r="4.5" class="dot3"/>')
    for p in fit["rejected"]:
        body.append(f'<circle cx="{px(p["v_ias"]):.1f}" cy="{py(p["sink_sl"]):.1f}" '
                    f'r="4.5" class="dotrej"><title>rejected: '
                    f'{p["v_ias"]:.1f} m/s, probably flown in lift</title></circle>')
    bx, by = px(fit["v_bg"]), py(fit["sink_bg"])
    tan_y = fit["sink_bg"] / fit["v_bg"] * x1
    body.append(f'<line x1="{px(0):.1f}" y1="{py(0):.1f}" '
                f'x2="{px(x1):.1f}" y2="{py(min(tan_y, y1)):.1f}" '
                f'class="tangent"/>')
    body.append(f'<text x="{px(0) + 6:.1f}" y="{py(0) + 13:.1f}" class="ann">'
                f'tangent from origin</text>')
    body.append(f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="5" class="mark3"/>')

    leg = [("measured + fit", "3"), ("design", "des")]
    ly = mt + 14
    for i, (lab, sl) in enumerate(leg):
        body.append(f'<line x1="{W - mr + 8}" y1="{ly + i * 20}" '
                    f'x2="{W - mr + 26}" y2="{ly + i * 20}" class="s{sl}" '
                    f'stroke-width="2"'
                    + (' stroke-dasharray="5 4"' if sl == "des" else "") + '/>')
        body.append(f'<text x="{W - mr + 31}" y="{ly + i * 20 + 4}" '
                    f'class="dlab">{lab}</text>')
    yy = ly + len(leg) * 20 + 10
    body.append(f'<circle cx="{W - mr + 17}" cy="{yy}" r="5" class="mark3"/>')
    body.append(f'<text x="{W - mr + 31}" y="{yy + 4}" class="dlab f3">'
                f'L/D {fit["ld_max"]:.1f} at {fit["v_bg"]:.1f} m/s</text>')
    ly += 10
    if fit["rejected"]:
        yy = ly + len(leg) * 20 + 20
        body.append(f'<circle cx="{W - mr + 17}" cy="{yy}" r="4.5" class="dotrej"/>')
        body.append(f'<text x="{W - mr + 31}" y="{yy + 4}" class="dlab">'
                    f'rejected ({len(fit["rejected"])})</text>')

    return (f'<figure><figcaption class="ctitle">Measured sink polar'
            f'<span class="cunit">reduced to sea level, {fit["mass"]:.3f} kg</span>'
            f'</figcaption><svg viewBox="0 0 {W} {H}" class="chart" role="img" '
            f'aria-label="Measured sink polar, L/D max {fit["ld_max"]:.1f}">'
            + "".join(g) + "".join(body)
            + f'<text x="{ml}" y="{H - 4}" class="axl">AIRSPEED  m/s</text>'
            + f'<text x="13" y="{mt + 10}" class="axl" '
              f'transform="rotate(-90 13 {mt + 10})" text-anchor="end">SINK  m/s</text>'
            + '</svg></figure>')


def chart_hist(vals, title, unit, slot=1, bins=22):
    if not vals:
        return ""
    W, H, ml, mr, mt, mb = 480, 250, 48, 16, 16, 40
    lo, hi = min(vals), max(vals)
    if hi <= lo:
        hi = lo + 1
    bw = (hi - lo) / bins
    counts = [0] * bins
    for v in vals:
        counts[min(bins - 1, int((v - lo) / bw))] += 1
    cmax = max(counts) or 1

    def px(v):
        return ml + (v - lo) / (hi - lo) * (W - ml - mr)

    def py(c):
        return mt + (1 - c / cmax) * (H - mt - mb)

    g, body = [], []
    for v in _nice(lo, hi, 5):
        if lo <= v <= hi:
            g.append(f'<text x="{px(v):.1f}" y="{H - mb + 16}" '
                     f'text-anchor="middle" class="tick">{v:g}</text>')
    g.append(f'<line x1="{ml}" y1="{H - mb}" x2="{W - mr}" y2="{H - mb}" '
             f'class="axis"/>')
    for i, c in enumerate(counts):
        x = px(lo + i * bw)
        w = max(1.0, (W - ml - mr) / bins - 2.0)
        body.append(f'<rect x="{x:.1f}" y="{py(c):.1f}" width="{w:.1f}" '
                    f'height="{max(0.0, H - mb - py(c)):.1f}" rx="2" '
                    f'class="b{slot}"><title>{c} samples</title></rect>')
    if lo <= 0 <= hi:
        body.append(f'<line x1="{px(0):.1f}" y1="{mt}" x2="{px(0):.1f}" '
                    f'y2="{H - mb}" class="axis"/>')
    return (f'<figure><figcaption class="ctitle">{html.escape(title)}'
            f'<span class="cunit">{html.escape(unit)}</span></figcaption>'
            f'<svg viewBox="0 0 {W} {H}" class="chart" role="img" '
            f'aria-label="{html.escape(title)}">'
            + "".join(g) + "".join(body) + '</svg></figure>')


CSS = """
:root{--bg:#eef1f5;--sheet:#fff;--sheet2:#f5f8fa;--ink:#131a21;--ink2:#465562;
--ink3:#7b8b99;--rule:#cfd8e1;--rules:#94a4b3;--grid:#dde4ea;--good:#2c6742;
--warn:#8a5a12;--bad:#a3341f;
--c0:#BD4E14;--c1:#0C87AA;--c2:#A16D00;--c3:#7C3B99;--cdes:#8593a0}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){
--bg:#0b1015;--sheet:#131b21;--sheet2:#182027;--ink:#e3eaf0;--ink2:#9bacb9;
--ink3:#6d8090;--rule:#25303a;--rules:#3d4f5c;--grid:#222d36;--good:#6dbb8c;
--warn:#d2a748;--bad:#e0765c;
--c0:#D2622A;--c1:#1D9AC0;--c2:#B98110;--c3:#9A57B5;--cdes:#6d8090}}
:root[data-theme=dark]{--bg:#0b1015;--sheet:#131b21;--sheet2:#182027;
--ink:#e3eaf0;--ink2:#9bacb9;--ink3:#6d8090;--rule:#25303a;--rules:#3d4f5c;
--grid:#222d36;--good:#6dbb8c;--warn:#d2a748;--bad:#e0765c;
--c0:#D2622A;--c1:#1D9AC0;--c2:#B98110;--c3:#9A57B5;--cdes:#6d8090}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);line-height:1.6;
font:15px/1.6 "Barlow",-apple-system,"Segoe UI",Helvetica,Arial,sans-serif}
.sheet{max-width:1100px;margin:0 auto;background:var(--sheet);
border-left:1px solid var(--rule);border-right:1px solid var(--rule);min-height:100vh}
.pad{padding:0 clamp(16px,3.5vw,44px)}
h1,h2,h3{font-family:"Barlow Condensed","Barlow",sans-serif;font-weight:600;
margin:0;line-height:1.15;text-wrap:balance}
h1{font-size:clamp(26px,4.2vw,40px)}
h2{font-size:clamp(20px,2.7vw,27px);margin-top:6px}
h3{font-size:15px;text-transform:uppercase;letter-spacing:.05em;color:var(--ink3)}
p{margin:0 0 .9em;max-width:74ch}
.mono,td.n,th.n,.tick,.dlab,.ann,.axl{font-family:"IBM Plex Mono",ui-monospace,
SFMono-Regular,Menlo,monospace;font-variant-numeric:tabular-nums}
header{padding-top:34px;padding-bottom:18px;border-bottom:1px solid var(--rule)}
.eyebrow{font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.16em;
text-transform:uppercase;color:var(--ink3);display:flex;align-items:center;gap:12px;
margin:0 0 12px}
.eyebrow::after{content:"";flex:1;height:1px;background:var(--rule)}
section{padding:30px 0;border-bottom:1px solid var(--rule)}
section:last-of-type{border-bottom:0}
.strip{display:grid;grid-template-columns:repeat(auto-fit,minmax(124px,1fr));
border-bottom:1px solid var(--rule)}
.strip div{padding:13px clamp(10px,2vw,18px);border-left:1px solid var(--rule)}
.strip div:first-child{border-left:0}
.strip dt{font-family:"IBM Plex Mono",monospace;font-size:10px;letter-spacing:.11em;
text-transform:uppercase;color:var(--ink3);margin:0 0 3px}
.strip dd{margin:0;font-family:"IBM Plex Mono",monospace;font-size:19px;
font-weight:500;font-variant-numeric:tabular-nums}
.strip dd small{font-size:11.5px;color:var(--ink2);font-weight:400}
table{border-collapse:collapse;width:100%;font-size:14px}
.scroll{overflow-x:auto;margin:18px 0}
th,td{padding:7px 14px 7px 0;text-align:left;border-bottom:1px solid var(--rule);
white-space:nowrap}
th{font-family:"IBM Plex Mono",monospace;font-size:10px;letter-spacing:.1em;
text-transform:uppercase;color:var(--ink3);font-weight:500;
border-bottom:1px solid var(--rules)}
td.n,th.n{text-align:right;padding-right:20px}
tr.hi td{background:var(--sheet2)}
.flag{font-family:"IBM Plex Mono",monospace;font-size:10px;letter-spacing:.07em;
padding:1px 6px;border:1px solid currentColor;text-transform:uppercase}
.fg{color:var(--good)}.fw{color:var(--warn)}.fb{color:var(--bad)}
figure{margin:22px 0}
.ctitle{font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.11em;
text-transform:uppercase;color:var(--ink3);margin-bottom:8px;display:flex;
justify-content:space-between;gap:12px;border-bottom:1px solid var(--rule);
padding-bottom:6px}
.cunit{color:var(--ink3);text-transform:none;letter-spacing:.03em}
svg.chart{display:block;width:100%;height:auto;overflow:visible}
.grid{stroke:var(--grid);stroke-width:1}
.axis{stroke:var(--rules);stroke-width:1}
.tick{font-size:10.5px;fill:var(--ink3)}
.axl{font-size:9.5px;letter-spacing:.13em;fill:var(--ink3)}
.dlab{font-size:11px;fill:var(--ink2)}
.ann{font-size:11px;fill:var(--ink2)}
.s0{stroke:var(--c0)}.s1{stroke:var(--c1)}.s2{stroke:var(--c2)}.s3{stroke:var(--c3)}
.sdes{stroke:var(--cdes)}
.f0{fill:var(--c0)}.f1{fill:var(--c1)}.f2{fill:var(--c2)}.f3{fill:var(--c3)}
.b0{fill:var(--c0)}.b1{fill:var(--c1)}.b2{fill:var(--c2)}.b3{fill:var(--c3)}
.dot3{fill:var(--c3);stroke:var(--sheet);stroke-width:2}
.dotrej{fill:none;stroke:var(--ink3);stroke-width:1.5;stroke-dasharray:2 2}
.mark3{fill:none;stroke:var(--c3);stroke-width:2}
.tangent{stroke:var(--ink3);stroke-width:1;stroke-dasharray:4 4}
.p0{fill:var(--c0)}.p1{fill:var(--c1)}.p2{fill:var(--c2)}.p3{fill:var(--c3)}
.xhair{stroke:var(--rules);stroke-width:1;pointer-events:none}
.cwrap{position:relative}
.tip{position:absolute;pointer-events:none;background:var(--sheet);
border:1px solid var(--rules);padding:6px 9px;font-family:"IBM Plex Mono",monospace;
font-size:11px;line-height:1.55;white-space:nowrap;transform:translate(-50%,-115%);
box-shadow:0 2px 8px rgba(0,0,0,.12)}
.tip b{font-weight:500}
.swatch{display:inline-block;width:10px;height:10px;margin-right:6px;
vertical-align:-1px}
.legend{display:flex;flex-wrap:wrap;gap:6px 20px;font-family:"IBM Plex Mono",
monospace;font-size:11px;color:var(--ink2);margin:-6px 0 10px}
.note{border-left:3px solid var(--warn);padding:2px 0 2px 16px;margin:18px 0;
max-width:72ch}
.note b{display:block;font-family:"IBM Plex Mono",monospace;font-size:10.5px;
letter-spacing:.11em;text-transform:uppercase;color:var(--warn);margin-bottom:4px}
details{margin:14px 0}summary{cursor:pointer;font-family:"IBM Plex Mono",monospace;
font-size:11px;letter-spacing:.1em;text-transform:uppercase;color:var(--ink3)}
footer{padding:26px 0 40px;color:var(--ink3);font-size:12.5px}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
"""

HOVER_JS = """
document.querySelectorAll('.cwrap[data-series]').forEach(w=>{
  const data=JSON.parse(w.dataset.series), svg=w.querySelector('svg'),
        tip=w.querySelector('.tip'), xh=w.querySelector('.xhair');
  if(!data.pts.length) return;
  function move(ev){
    const r=svg.getBoundingClientRect();
    const vb=svg.viewBox.baseVal;
    const vx=(ev.clientX-r.left)/r.width*vb.width;
    let best=0,bd=1e9;
    for(let i=0;i<data.pts.length;i++){
      const d=Math.abs(data.pts[i][0]-vx); if(d<bd){bd=d;best=i;}
    }
    const p=data.pts[best];
    xh.setAttribute('x1',p[0]); xh.setAttribute('x2',p[0]);
    xh.style.display='';
    tip.hidden=false;
    tip.style.left=((p[0]/vb.width)*r.width)+'px';
    tip.style.top=(r.height*0.42)+'px';
    tip.innerHTML='<b>'+(p[1]/60).toFixed(1)+' min</b>'+
      p[2].map((v,k)=>v===null?'':'<br><span class="swatch" style="background:var(--c'
        +data.slots[k]+')"></span>'+data.labels[k]+' <b>'+v.toFixed(data.dp[k])
        +'</b> '+data.unit).join('');
  }
  w.addEventListener('pointermove',move);
  w.addEventListener('pointerleave',()=>{tip.hidden=true;xh.style.display='none';});
});
"""


def _tbl(rows, head):
    h = "".join(f'<th class="{"n" if str(c).startswith("~") else ""}">'
                f'{html.escape(str(c).lstrip("~"))}</th>' for c in head)
    b = ""
    for r in rows:
        cls = ' class="hi"' if r and str(r[0]).startswith("*") else ""
        cells = ""
        for i, c in enumerate(r):
            c = str(c)
            if i == 0:
                c = c.lstrip("*")
            nn = ' class="n"' if str(head[i]).startswith("~") else ""
            cells += f"<td{nn}>{c}</td>"
        b += f"<tr{cls}>{cells}</tr>"
    return f'<div class="scroll"><table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'


def build_report(tab, lg, fit, bs, cv, terms, pred, thermals, energy,
                 mass_kg, used_gs, out_path):
    t = tab["t"]
    t0 = t[0] if t else 0.0
    trel = [x - t0 for x in t]
    tspan = (0.0, trel[-1] if trel else 1.0)
    airborne = thermals["airborne_s"] if thermals else 0.0
    tot = phase_totals(tab)
    g = AN.geom()
    m = AN.mass_rollup()

    def ser(key, scale=1.0):
        if key not in tab:
            return []
        return [(a, v * scale) for a, v in _dsample(trel, tab[key])]

    phases = [(s["mode"], s["t0"] - t0, s["t1"] - t0) for s in tab["segments"]]

    charts = []
    if "alt" in tab:
        charts.append(chart_time([("altitude", ser("alt"), 3)], "ALTITUDE m",
                                 "Altitude and flight phase", phases=phases,
                                 tspan=tspan, unit="m", h=260, cid="alt"))
        charts.append('<div class="legend">' + "".join(
            f'<span><span class="swatch p{PHASE_SLOT[p]}"></span>'
            f'{PHASE_LABEL[p]} &mdash; {tot.get(p, 0) / 60:.0f} min</span>'
            for p in PHASE_ORDER if tot.get(p)) + '</div>')
    if "airspeed" in tab:
        charts.append(chart_time([("airspeed", ser("airspeed"), 1)],
                                 "AIRSPEED m/s", "Airspeed", tspan=tspan,
                                 unit="m/s", h=200, cid="asp"))

    pw = []
    for key, lab, slot in (("solar_w", "solar in", 2),
                           ("turbine_w", "turbine in", 1),
                           ("p_batt", "battery net out", 0)):
        s = ser(key)
        if s:
            pw.append((lab, s, slot))
    if pw:
        charts.append(chart_time(pw, "POWER W", "Electrical power",
                                 tspan=tspan, unit="W", h=230, cid="pwr"))
    en = []
    for key, lab, slot in (("wh_solar_w", "solar", 2),
                           ("wh_turbine_w", "turbine", 1),
                           ("wh_p_batt", "battery net", 0)):
        s = ser(key)
        if s:
            en.append((lab, s, slot))
    if en:
        charts.append(chart_time(en, "ENERGY Wh", "Cumulative energy",
                                 tspan=tspan, unit="Wh", h=210, cid="wh"))

    side = []
    if fit:
        side.append(chart_polar(fit, lambda v: AN.glide_sink_at(
            v, m["mtow_g"] / 1000.0 * G)))
    if thermals and thermals.get("hist"):
        side.append(chart_hist(thermals["hist"], "Climb rate while thermalling",
                               "m/s", slot=1))

    # ---- key-figure strip ----
    strip = [("Airborne", f"{airborne / 3600:.2f}", "h"),
             ("Soaring", f"{100 * (tot.get('thermal', 0) + tot.get('glide', 0)) / max(airborne, 1):.0f}",
              "% unpowered")]
    if fit:
        strip += [("L/D max", f"{fit['ld_max']:.1f}", "measured"),
                  ("Min sink", f"{fit['sink_min']:.2f}", "m/s"),
                  ("Drag vs design", f"{pred['drag_scale']['scale']:.2f}", "x C_D0")]
    if energy.get("solar_in"):
        strip.append(("Solar", f"{energy['solar_in']:.0f}", "Wh"))
    if energy.get("turbine_in"):
        strip.append(("Turbine", f"{energy['turbine_in']:.1f}", "Wh"))
    strip_html = "".join(f"<div><dt>{k}</dt><dd>{v} <small>{u}</small></dd></div>"
                         for k, v, u in strip)

    # ---- sections ----
    body = []
    if fit:
        ok_ld = pred["identifiable"]["ld_max"]
        rows = [[f"{r['v']:.0f} m/s", f"{r['measured']:.3f}", f"{r['design']:.3f}",
                 f"{r['delta_pct']:+.1f} %"] for r in pred["sink_cmp"]]
        body.append(f"""
<section><div class="eyebrow">Measured vs design</div>
<h2>The sink polar</h2>
<p>{fit['n_used']} of {fit['n_total']} stabilised glide legs, reduced to
sea-level density at {mass_kg:.3f} kg. A {terms}-term fit was chosen by
leave-one-out cross-validation
({', '.join(f'{k} terms: {v:.4f} m/s RMS' for k, v in sorted(cv.items()))}).
R&sup2; {fit['r2']:.4f}, residual scatter {fit['resid_sd']:.3f} m/s.</p>
{_tbl(rows, ["Airspeed", "~Measured sink", "~Design sink", "~Difference"])}
<p>That comparison makes no assumptions &mdash; both curves are simply
evaluated at the same speeds. The table below fits a drag level to it.</p>
""" + _tbl([
            ["*L/D max", f"{fit['ld_max']:.2f}", f"{pred['design']['ld_max']:.2f}",
             f"{pred['delta'].get('ld_max', 0):+.1f} %",
             f'<span class="flag {"fg" if ok_ld[0] else "fw"}">{ok_ld[1]}</span>'],
            ["Speed for best glide", f"{fit['v_bg']:.2f} m/s",
             f"{pred['design']['v_bg']:.2f} m/s",
             f"{pred['delta'].get('v_bg', 0):+.1f} %",
             f'<span class="flag {"fg" if pred["identifiable"]["v_bg"][0] else "fw"}">'
             f'{pred["identifiable"]["v_bg"][1]}</span>'],
            ["Minimum sink", f"{fit['sink_min']:.3f} m/s",
             f"{pred['design']['sink_min']:.3f} m/s",
             f"{pred['delta'].get('sink_min', 0):+.1f} %", ""],
            ["*Drag level vs design",
             f"&times;{pred['drag_scale']['scale']:.3f}"
             + (f" [{pred['scale_lo']:.3f}&ndash;{pred['scale_hi']:.3f}]"
                if pred['scale_lo'] else ""),
             "&times;1.000", "", ""],
        ], ["Quantity", "~Measured", "~Design", "~Delta", "Confidence"])
            + "</section>")

        cd0_ok, cd0_why = pred["identifiable"]["cd0"]
        e_ok, e_why = pred["identifiable"]["e"]
        b_cd0, b_e = bs.get("cd0"), bs.get("e")
        body.append(f"""
<section><div class="eyebrow">Coefficients</div>
<h2>Splitting parasite from induced drag</h2>
<p>A sink curve determines <em>total</em> drag well. Separating it into a
parasite term and an induced term is a different question, and often the data
cannot answer it: the basis functions V&sup3;, V and 1/V are nearly collinear
over a narrow speed range, so wildly different coefficient pairs fit the same
curve. The 10&ndash;90 % intervals below come from resampling the glide legs
300 times.</p>
""" + _tbl([
            ["C_D0 (equivalent)",
             f"{fit['cd0']:.4f}" if fit.get('cd0') is not None else "&mdash;",
             (f"{b_cd0['lo']:.4f} &ndash; {b_cd0['hi']:.4f}" if b_cd0 else "&mdash;"),
             f"{pred['design']['cd0']:.4f}",
             f'<span class="flag {"fg" if cd0_ok else "fb"}">'
             f'{"usable" if cd0_ok else "not identifiable"}</span> {cd0_why}'],
            ["Oswald e",
             f"{fit['e']:.3f}" if fit.get('e') else "&mdash;",
             (f"{b_e['lo']:.3f} &ndash; {b_e['hi']:.3f}" if b_e else "&mdash;"),
             f"{pred['design']['e']:.3f}",
             f'<span class="flag {"fg" if e_ok else "fb"}">'
             f'{"usable" if e_ok else "not identifiable"}</span> {e_why}'],
        ], ["Coefficient", "~Fitted", "~10-90 % interval", "~Design", "Verdict"])
            + """
<div class="note"><b>If these came back "not identifiable"</b>
Fly a wider speed range. Legs from just above stall to about 2.5&times; stall,
at seven or more speeds, three repeats each, in the calmest air you can find
&mdash; early morning or late evening. The single biggest error source is
flying a &quot;still air&quot; leg through weak lift, which reads as
impossibly low sink; the fitter drops points more than 2&sigma; below the
curve for that reason, and they are shown as hollow circles on the polar.
Until the interval tightens, use the drag-level factor above: it is one
parameter fitted to one well-determined quantity, so it is always
identifiable.</div></section>
""")

    if thermals and thermals.get("n"):
        hm = pred.get("harvest_measured")
        hrows = [["Mean climb while thermalling", f"{thermals['mean_climb']:.2f} m/s"],
                 ["Best climb", f"{thermals['best_climb']:.2f} m/s"],
                 ["90th percentile climb", f"{thermals['p90_climb']:.2f} m/s"],
                 ["Time in lift", f"{100 * thermals['f_lift']:.0f} % of airborne time"]]
        if hm:
            hrows.append(["*Predicted turbine harvest, this day",
                          f"{hm['harvest_w']:.1f} W sustained"
                          + (f", descending at {hm['v']:.0f} m/s" if hm.get("v") else "")])
        body.append(f"""
<section><div class="eyebrow">Soaring</div>
<h2>What the air was doing, and what the turbine could take</h2>
<p>Thermal strength and how much of the flight was spent in lift are the two
inputs the turbine harvest model needs. Both are measured here rather than
assumed.</p>
{_tbl(hrows, ["Measure", "~Value"])}
</section>""")

    erows = []
    for site in pred["endurance_design"]:
        erows.append([site, f"{pred['endurance_design'][site]:.2f} h",
                      f"{pred['endurance_measured'][site]:.2f} h",
                      f"{pred['endurance_measured'][site] - pred['endurance_design'][site]:+.2f} h"])
    body.append(f"""
<section><div class="eyebrow">Prediction</div>
<h2>Endurance, re-run on the measured drag level</h2>
<p>The design endurance model from <code>tools/analysis.py</code>, re-run with
C_D0 scaled by the factor fitted above. Everything else &mdash; array, pack,
hotel load, thermal model &mdash; is unchanged, so the difference is the drag
finding alone.</p>
{_tbl(erows, ["Site", "~On design drag", "~On measured drag", "~Change"])}
</section>""")

    if energy:
        erow = [[k.replace("_", " "), f"{v:.1f} Wh"] for k, v in energy.items()]
        body.append(f"""
<section><div class="eyebrow">Energy</div>
<h2>What went in and out</h2>
{_tbl(erow, ["Channel", "~Total"])}
</section>""")

    notes = "".join(f"<li>{html.escape(n)}</li>" for n in lg.notes)
    warn = ""
    if used_gs:
        warn = """<div class="note"><b>No airspeed in this log</b>
Groundspeed was used instead, so every polar number is contaminated by wind
&mdash; a 3 m/s wind biases L/D by tens of percent depending on heading. Fit a
pitot and airspeed sensor before trusting any of the polar results.</div>"""
    if fit and fit["n_total"] < 6:
        warn += f"""<div class="note"><b>Only {fit['n_total']} usable glide legs</b>
The fit will be poorly conditioned. Fly a dedicated polar sortie: wings level,
motor off, hold each speed for 30 s, seven or more speeds.</div>"""

    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FF-1 flight {os.path.basename(lg.source)}</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@600&family=Barlow:wght@400;500&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>{CSS}</style></head><body><div class="sheet">
<header class="pad">
<div class="eyebrow">FF-1 Ember &middot; flight analysis</div>
<h1>{html.escape(os.path.basename(lg.source))}</h1>
<p style="color:var(--ink2);margin-top:8px">{airborne / 60:.0f} minutes airborne,
{len(t):,} samples at {1 / tab['dt']:.0f} Hz, analysed at {mass_kg:.3f} kg.</p>
</header>
<dl class="strip">{strip_html}</dl>
<main class="pad">
{warn}
<section><div class="eyebrow">Flight trace</div>{''.join(charts)}</section>
<section><div class="eyebrow">Polar and thermals</div>{''.join(side)}</section>
{''.join(body)}
<footer><details><summary>Log parsing notes</summary><ul>{notes}</ul></details>
<p style="margin-top:14px">Generated by <code>tools/flight_analysis.py</code>.
Design reference: <code>tools/analysis.py</code> and
<code>docs/ANALYSIS.md</code>.</p></footer>
</main></div><script>{HOVER_JS}</script></body></html>"""


# ==========================================================================
# CLI
# ==========================================================================

def run(path, mass_kg=None, dt=0.5, out=None, win_s=20.0):
    lg = FL.read(path)
    tab = lg.resample(dt)
    if not tab.get("t"):
        raise SystemExit("log contains no usable samples")
    m = AN.mass_rollup()
    mass_kg = mass_kg or m["mtow_g"] / 1000.0
    derive(tab, mass_kg)
    classify(tab)
    g = AN.geom()

    pts, used_gs = glide_windows(tab, win_s=win_s)
    fit = bs = pred = None
    terms, cv = 2, {}
    if len(pts) >= 4:
        terms, cv = choose_terms(pts, mass_kg, g["S"], g["AR"])
        fit = fit_polar(pts, mass_kg, g["S"], g["AR"], terms=terms)
        if fit:
            bs = bootstrap(pts, mass_kg, g["S"], g["AR"], terms)
    th = thermal_stats(tab)
    energy = energy_accounting(tab)
    if fit:
        pred = predict(fit, tab, th, mass_kg, bs)
    else:
        pred = {"design": {}, "delta": {}, "identifiable": {},
                "sink_cmp": [], "drag_scale": {"scale": 1.0, "cd0_eff": AN.cd0_clean()},
                "scale_lo": None, "scale_hi": None,
                "endurance_design": {}, "endurance_measured": {}}

    out = out or os.path.join(ROOT, "reports",
                              os.path.splitext(os.path.basename(path))[0] + ".html")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as fh:
        fh.write(build_report(tab, lg, fit, bs or {}, cv, terms, pred, th,
                              energy, mass_kg, used_gs, out))

    print(f"wrote {os.path.relpath(out, ROOT)} "
          f"({os.path.getsize(out) / 1024:.0f} kB)")
    tot = phase_totals(tab)
    print(f"  {th['airborne_s'] / 60:.0f} min airborne  "
          + "  ".join(f"{k} {v / 60:.0f}m" for k, v in sorted(tot.items())))
    if fit:
        print(f"  polar: {fit['n_used']}/{fit['n_total']} legs, {terms}-term, "
              f"R2 {fit['r2']:.3f}")
        print(f"  L/D max {fit['ld_max']:.2f} at {fit['v_bg']:.2f} m/s, "
              f"min sink {fit['sink_min']:.3f} m/s")
        print(f"  drag level x{pred['drag_scale']['scale']:.3f} vs design")
        for k, (ok, why) in pred["identifiable"].items():
            print(f"    {k:<8s} {'usable' if ok else 'NOT identifiable'} ({why})")
    else:
        print(f"  no drag polar: only {len(pts)} stabilised glide legs found")
    return out


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 0
    path = argv[1]
    mass = dt = out = None
    win = 20.0
    do_open = False
    i = 2
    while i < len(argv):
        a = argv[i]
        if a == "--mass":
            mass = float(argv[i + 1]); i += 2
        elif a == "--dt":
            dt = float(argv[i + 1]); i += 2
        elif a in ("-o", "--out"):
            out = argv[i + 1]; i += 2
        elif a == "--window":
            win = float(argv[i + 1]); i += 2
        elif a == "--open":
            do_open = True; i += 1
        else:
            print(f"unknown option: {a}")
            return 2
    p = run(path, mass_kg=mass, dt=dt or 0.5, out=out, win_s=win)
    if do_open:
        import webbrowser
        webbrowser.open("file://" + os.path.abspath(p))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
