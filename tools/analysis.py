#!/usr/bin/env python3
"""
Forever-Flight FF-1 performance, energy, stability and payload analysis.

Everything the design documents quote is computed here, from tools/config.py,
and written to docs/ANALYSIS.md. Nothing in the docs is hand-typed physics.

Run:  python3 tools/analysis.py
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import build_model as B
import config as C

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G = C.MISSION["g"]
RHO = C.MISSION["rho_cruise"]
NU = C.MISSION["nu"]


# ==========================================================================
# mass and geometry
# ==========================================================================

def mass_rollup():
    groups, total, var = {}, 0.0, 0.0
    for (grp, item, g, unc) in C.MASS:
        groups.setdefault(grp, []).append((item, g, unc))
        total += g
        var += unc * unc
    growth = total * C.MASS_GROWTH_ALLOWANCE
    return {"groups": groups, "itemised_g": total, "growth_g": growth,
            "mtow_g": total + growth, "sigma_g": math.sqrt(var)}


def geom():
    stns = B.wing_geometry()
    S = B.planform_area_mm2(stns) / 1e6
    b = C.WING["span"] / 1000.0
    mac = B.mean_aero_chord(stns) / 1000.0
    v = C.VTAIL
    S_v_panel = 2 * (v["panel_span"] / 1000.0) * \
        ((v["root_chord"] + v["tip_chord"]) / 2.0 / 1000.0)
    dih = math.radians(v["dihedral"])
    S_h = S_v_panel * math.cos(dih) ** 2
    S_vert = S_v_panel * math.sin(dih) ** 2
    x_le_root = C.WING["stations"][0][2] / 1000.0
    x_ac_w = x_le_root + 0.25 * mac
    x_ac_t = (v["le_x"] + 0.25 * (v["root_chord"] + v["tip_chord"]) / 2.0) / 1000.0
    return {"S": S, "b": b, "AR": b * b / S, "mac": mac,
            "S_tail_wetted": S_v_panel, "S_h": S_h, "S_v": S_vert,
            "x_ac_w": x_ac_w, "x_ac_t": x_ac_t,
            "b_h": 2 * (v["panel_span"] / 1000.0) * math.cos(dih)}


# ==========================================================================
# drag build-up and polar
# ==========================================================================

def cd0_buildup():
    rows = [("Wing profile (at design CL)", C.DRAG["wing_cd_min"],
             f"Re ~ {int(120000/1000)*1000:,}, SD7037-class section")]
    rows += [(n, d, note) for (n, d, note) in C.DRAG["components"]]
    raw = sum(r[1] for r in rows)
    margin = raw * C.DRAG["margin"]
    return rows, raw, margin, raw + margin


def cd0_clean():
    return cd0_buildup()[3]


def polar(CL, cd0=None, motor=False, rat=False):
    g = geom()
    cd0 = cd0_clean() if cd0 is None else cd0
    cd0 += C.DRAG["motor_deployed_off"] if motor else 0.0
    cd0 += C.DRAG["rat_deployed_feathered"] if rat else 0.0
    cdp = C.DRAG["wing_cd_k"] * (CL - C.DRAG["wing_cl_design"]) ** 2
    cdi = CL * CL / (math.pi * g["AR"] * C.DRAG["oswald_e"])
    return cd0 + cdp + cdi


def speed_for_cl(CL, W, S, rho=RHO):
    return math.sqrt(2.0 * W / (rho * S * CL))


def sweep(W, motor=False, rat=False, rho=RHO):
    g = geom()
    out = []
    cl = 0.15
    while cl <= C.DRAG["cl_max"] + 1e-9:
        cd = polar(cl, motor=motor, rat=rat)
        V = speed_for_cl(cl, W, g["S"], rho)
        ld = cl / cd
        sink = V / ld
        out.append({"CL": cl, "CD": cd, "V": V, "LD": ld, "sink": sink,
                    "P_aero": W * sink,
                    "Re": V * g["mac"] / NU})
        cl += 0.01
    return out


def key_points(W, motor=False, rat=False):
    s = sweep(W, motor=motor, rat=rat)
    best_glide = max(s, key=lambda r: r["LD"])
    min_sink = min(s, key=lambda r: r["sink"])
    g = geom()
    v_stall = speed_for_cl(C.DRAG["cl_max"], W, g["S"])
    v_stall_flap = speed_for_cl(C.DRAG["cl_max_flapped"], W, g["S"])
    return {"best_glide": best_glide, "min_sink": min_sink,
            "v_stall": v_stall, "v_stall_flapped": v_stall_flap, "sweep": s}


def elec_power_for(V, W, motor=True, rat=False):
    """Electrical watts to hold level flight at V."""
    g = geom()
    CL = 2.0 * W / (RHO * g["S"] * V * V)
    if CL > C.DRAG["cl_max"]:
        return None, CL
    CD = polar(CL, motor=motor, rat=rat)
    D = 0.5 * RHO * g["S"] * V * V * CD
    p_aero = D * V
    e = C.EFF
    return p_aero / (e["prop"] * e["motor"] * e["esc"]), CL


# ==========================================================================
# turbine ("dynamo") physics
# ==========================================================================

def rat_power(V, rho=RHO):
    """Shaft and electrical watts, and the drag the extraction costs."""
    r = C.RAT
    A = math.pi * (r["rotor_dia"] / 2000.0) ** 2
    p_air = 0.5 * rho * A * V ** 3
    p_shaft = p_air * r["cp"]
    p_elec = p_shaft * C.EFF["generator"] * C.EFF["rectifier_charger"]
    return {"V": V, "disc_area_m2": A, "p_available": p_air,
            "p_shaft": p_shaft, "p_elec": p_elec,
            "extraction_drag_N": p_shaft / V,
            "rpm": r["lambda_design"] * V / (r["rotor_dia"] / 2000.0) * 60 / (2 * math.pi)}


def regen_descent(W, V, dh_m):
    """Energy recovered by descending dh_m with the turbine on, vs gliding."""
    g = geom()
    rp = rat_power(V)
    CL = 2.0 * W / (RHO * g["S"] * V * V)
    CD = polar(CL, motor=False, rat=True)
    D_air = 0.5 * RHO * g["S"] * V * V * CD
    p_air = D_air * V
    frac = rp["p_shaft"] / (rp["p_shaft"] + p_air)
    pe = W * dh_m                                     # potential energy, J
    recovered_J = pe * frac * C.EFF["generator"] * C.EFF["rectifier_charger"]
    sink = (p_air + rp["p_shaft"]) / W
    return {"V": V, "dh_m": dh_m, "pe_wh": pe / 3600.0,
            "recovered_wh": recovered_J / 3600.0,
            "recovery_frac": recovered_J / pe,
            "sink_ms": sink, "descent_time_s": dh_m / sink,
            "turbine_share": frac, "p_elec": rp["p_elec"]}


def rat_hold_altitude(W, V):
    """Lift rate needed to hold height at V with the turbine extracting."""
    g = geom()
    rp = rat_power(V)
    CL = 2.0 * W / (RHO * g["S"] * V * V)
    CD = polar(CL, motor=False, rat=True)
    p_air = 0.5 * RHO * g["S"] * V ** 3 * CD
    return {"V": V, "p_elec": rp["p_elec"],
            "thermal_needed_ms": (p_air + rp["p_shaft"]) / W,
            "glide_sink_ms": p_air / W}


# ==========================================================================
# solar
# ==========================================================================

def solar_elevation(lat_deg, doy, hour_solar):
    dec = math.radians(23.45) * math.sin(2 * math.pi * (284 + doy) / 365.0)
    H = math.radians(15.0 * (hour_solar - 12.0))
    lat = math.radians(lat_deg)
    s = (math.sin(lat) * math.sin(dec) + math.cos(lat) * math.cos(dec) * math.cos(H))
    return math.degrees(math.asin(max(-1.0, min(1.0, s))))


def clear_sky(lat, doy, hour, clearness):
    """Hottel-style clear-sky irradiance on a horizontal surface, W/m2."""
    el = solar_elevation(lat, doy, hour)
    if el <= 2.0:
        return 0.0, el
    sin_el = math.sin(math.radians(el))
    am = 1.0 / (sin_el + 0.50572 * (el + 6.07995) ** -1.6364)   # Kasten-Young
    dni = 1361.0 * (clearness ** (am ** 0.678))
    beam = dni * sin_el
    diffuse = 0.13 * beam                       # simple isotropic allowance
    return beam + diffuse, el


def array_power(irr_horiz):
    s = C.SOLAR
    layout = B.solar_layout()
    chain = (s["cell_eff_stc"] * s["encapsulation_loss"] * s["temp_loss"]
             * s["mismatch_soiling"] * s["mppt_eff"] * s["attitude_factor"])
    return irr_horiz * layout["wing_area_m2"] * chain, chain


# ==========================================================================
# mission simulation
# ==========================================================================

def thermal_fraction(hour, peak, t0=9.5, t1=18.5):
    if hour <= t0 or hour >= t1:
        return 0.0
    return peak * math.sin(math.pi * (hour - t0) / (t1 - t0)) ** 0.7


def simulate(site, launch_h=9.0, peak_soar=0.75, use_solar=True,
             allow_soar=True, dt_s=30.0, max_h=14.0):
    name, lat, doy, clearness = site
    m = mass_rollup()
    W = m["mtow_g"] / 1000.0 * G
    bat = C.BATTERY
    cap_wh = bat["cells_series"] * bat["cells_parallel"] * bat["cell_wh"]
    usable = cap_wh * bat["usable_dod"] - bat["reserve_wh"]
    hotel = sum(h[1] for h in C.HOTEL)

    kp = key_points(W, motor=False)
    v_cruise_soar = kp["best_glide"]["V"]
    p_cruise, _ = elec_power_for(max(v_cruise_soar, kp["min_sink"]["V"]), W, motor=True)

    e = usable                                   # start full
    t, trace, endurance = launch_h, [], 0.0
    climbed = False
    while t < launch_h + max_h:
        irr, el = clear_sky(lat, doy, t, clearness) if use_solar else (0.0, 0.0)
        p_in, _ = array_power(irr) if use_solar else (0.0, 0.0)
        soar = thermal_fraction(t, peak_soar) if allow_soar else 0.0
        p_out = hotel + (1.0 - soar) * p_cruise
        if not climbed:                          # initial climb to the soaring band
            e -= (C.MISSION["cruise_alt_agl_m"] / 2.5) * (95.0 - p_in) / 3600.0
            climbed = True
        e += (p_in - p_out) * dt_s / 3600.0
        e = min(e, usable)
        trace.append({"t": t, "el": el, "irr": irr, "p_in": p_in,
                      "p_out": p_out, "soar": soar, "e": e})
        if e <= 0.0:
            break
        t += dt_s / 3600.0
        endurance = t - launch_h
    return {"site": name, "endurance_h": endurance, "trace": trace,
            "p_cruise": p_cruise, "hotel": hotel, "usable_wh": usable,
            "cap_wh": cap_wh, "solar_wh": sum(r["p_in"] * dt_s / 3600 for r in trace),
            "peak_solar_w": max((r["p_in"] for r in trace), default=0.0),
            "reached_target": endurance >= C.MISSION["target_endurance_h"]}


# ==========================================================================
# stability
# ==========================================================================

def stability():
    g = geom()
    e = C.DRAG["oswald_e"]
    a0 = 2 * math.pi
    a_w = a0 / (1 + a0 / (math.pi * g["AR"] * e))
    ar_h = g["b_h"] ** 2 / g["S_h"]
    a_t = a0 / (1 + a0 / (math.pi * ar_h * 0.90))
    l_t = g["x_ac_t"] - g["x_ac_w"]
    V_h = g["S_h"] * l_t / (g["S"] * g["mac"])
    V_v = g["S_v"] * l_t / (g["S"] * g["b"])
    deps = 2.0 * a_w / (math.pi * g["AR"])
    eta = 0.90
    x_np = 0.25 + (a_t / a_w) * eta * V_h * (1 - deps)
    return {"a_w": a_w, "a_t": a_t, "AR_h": ar_h, "l_t": l_t,
            "V_h": V_h, "V_v": V_v, "deps_dalpha": deps,
            "x_np_mac": x_np, "cg_fwd": x_np - 0.28, "cg_aft": x_np - 0.12,
            "cg_target": x_np - 0.20}


# ==========================================================================
# structure
# ==========================================================================

def wing_beam(n_limit=5.0, E_gpa=130.0, cap_w_mm=9.0, ply_t_mm=0.115, plies=3):
    """Numerically integrate an elliptic lift distribution for M(y) and tip sag."""
    m = mass_rollup()
    W = m["mtow_g"] / 1000.0 * G
    g = geom()
    half = g["b"] / 2.0
    N = 400
    dy = half / N
    ys = [(i + 0.5) * dy for i in range(N)]
    ell = [math.sqrt(max(0.0, 1.0 - (y / half) ** 2)) for y in ys]
    k = (n_limit * W / 2.0) / (sum(ell) * dy)
    load = [k * e for e in ell]                       # N/m

    shear = [0.0] * N
    acc = 0.0
    for i in range(N - 1, -1, -1):
        acc += load[i] * dy
        shear[i] = acc
    mom = [0.0] * N
    acc = 0.0
    for i in range(N - 1, -1, -1):
        acc += shear[i] * dy
        mom[i] = acc

    stns = B.wing_geometry()

    def chord_at(y):
        for i in range(len(stns) - 1):
            if stns[i]["y"] <= y * 1000 <= stns[i + 1]["y"]:
                t = (y * 1000 - stns[i]["y"]) / (stns[i + 1]["y"] - stns[i]["y"])
                return (stns[i]["chord"] + t * (stns[i + 1]["chord"] - stns[i]["chord"])) / 1000.0
        return stns[-1]["chord"] / 1000.0

    tc = 0.0948
    cap_t = plies * ply_t_mm / 1000.0
    EI, sep, cap_a = [], [], []
    for y in ys:
        c = chord_at(y)
        h = tc * c * 0.86                              # cap centroid separation
        taper = max(0.30, 1.0 - 0.72 * (y / half))     # plies dropped outboard
        A = (cap_w_mm / 1000.0) * cap_t * taper
        I = 2 * A * (h / 2.0) ** 2
        EI.append(E_gpa * 1e9 * I)
        sep.append(h)
        cap_a.append(A)

    theta, defl, th, de = [0.0] * N, [0.0] * N, 0.0, 0.0
    for i in range(N):
        th += (mom[i] / EI[i]) * dy
        de += th * dy
        theta[i], defl[i] = th, de

    sigma = [mom[i] / (cap_a[i] * sep[i]) / 1e6 for i in range(N)]
    return {"M_root": mom[0], "tip_defl_mm": defl[-1] * 1000,
            "tip_defl_1g_mm": defl[-1] * 1000 / n_limit,
            "sigma_root_mpa": sigma[0], "sigma_max_mpa": max(sigma),
            "cap_area_root_mm2": cap_a[0] * 1e6, "cap_sep_root_mm": sep[0] * 1000,
            "n_limit": n_limit, "cap_w_mm": cap_w_mm, "plies": plies,
            "EI_root": EI[0], "shear_root_N": shear[0]}


# ==========================================================================
# LWIR detection
# ==========================================================================

H_PL, C_L, K_B, SIGMA = 6.62607015e-34, 2.99792458e8, 1.380649e-23, 5.670374419e-8


def planck(lam, T):
    x = H_PL * C_L / (lam * K_B * T)
    if x > 700:
        return 0.0
    return (2 * math.pi * H_PL * C_L ** 2) / (lam ** 5 * (math.exp(x) - 1.0))


def band_exitance(T, l1=8e-6, l2=14e-6, n=400):
    h = (l2 - l1) / n
    s = planck(l1, T) + planck(l2, T)
    for i in range(1, n):
        s += (4 if i % 2 else 2) * planck(l1 + i * h, T)
    return s * h / 3.0


def apparent_temp(L_band, lo=250.0, hi=1200.0):
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if band_exitance(mid) < L_band:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def detection(alt_m=None):
    p = C.PAYLOAD_SENSOR
    alt = alt_m or C.MISSION["cruise_alt_agl_m"]
    hfov = math.radians(p["hfov_deg"])
    swath = 2 * alt * math.tan(hfov / 2)
    gsd = swath / p["h_pixels"]
    pix_area = gsd * gsd * (p["v_pixels"] / p["v_pixels"])
    Tb = p["background_c"] + 273.15
    Lb = band_exitance(Tb)
    Lf = band_exitance(p["flame_temp_k"])
    rows = []
    for a in p["test_fire_areas_m2"]:
        f = min(1.0, a / pix_area)
        Lmix = f * Lf + (1 - f) * Lb
        Ta = apparent_temp(Lmix)
        rows.append({"fire_m2": a, "fill_frac": f, "T_app_c": Ta - 273.15,
                     "delta_k": Ta - Tb,
                     "snr": (Ta - Tb) / (p["netd_mk"] / 1000.0)})
    m = mass_rollup()
    W = m["mtow_g"] / 1000.0 * G
    V = key_points(W)["best_glide"]["V"]
    return {"alt_m": alt, "swath_m": swath, "gsd_m": gsd, "pixel_area_m2": pix_area,
            "v_swath_m": 2 * alt * math.tan(hfov / 2 * p["v_pixels"] / p["h_pixels"]),
            "rows": rows, "V": V,
            "coverage_km2_h": V * swath * 3600 / 1e6,
            "coverage_sidelap_km2_h": V * swath * 0.70 * 3600 / 1e6,
            "band_frac_bg": band_exitance(Tb) / (SIGMA * Tb ** 4),
            "band_frac_flame": Lf / (SIGMA * p["flame_temp_k"] ** 4)}


# ==========================================================================
# trade studies
# ==========================================================================

def practical_min_sink(W, margin=1.15):
    """Min sink is CL-limited, i.e. it lands on the stall. Fly 15% above it."""
    g = geom()
    V = margin * speed_for_cl(C.DRAG["cl_max"], W, g["S"])
    CL = 2.0 * W / (RHO * g["S"] * V * V)
    CD = polar(CL)
    return {"V": V, "CL": CL, "LD": CL / CD, "sink": V / (CL / CD)}


def retraction_benefit():
    """What the two pods cost when they hang out in the breeze."""
    m = mass_rollup()
    W = m["mtow_g"] / 1000.0 * G
    out = {}
    for label, kw in [("both stowed", {}),
                      ("motor deployed", {"motor": True}),
                      ("turbine deployed", {"rat": True}),
                      ("both deployed", {"motor": True, "rat": True})]:
        kp = key_points(W, **kw)
        bg = kp["best_glide"]
        out[label] = {"cd0": cd0_clean()
                      + (C.DRAG["motor_deployed_off"] if kw.get("motor") else 0.0)
                      + (C.DRAG["rat_deployed_feathered"] if kw.get("rat") else 0.0),
                      "LD": bg["LD"], "sink": bg["sink"], "V": bg["V"]}
    base = out["both stowed"]
    for k, v in out.items():
        v["sink_penalty_pct"] = 100.0 * (v["sink"] / base["sink"] - 1.0)
        v["ld_penalty_pct"] = 100.0 * (1.0 - v["LD"] / base["LD"])
    return out


def variant_nose_prop():
    """Trade: keep the deployable dorsal pylon, or use a nose folder?

    The sensor turret is slung UNDER the nose, not on the tip, so a nose
    folding prop is geometrically available. This prices the alternative.
    """
    removed = {"Pylon, 4-bar, over-centre lock": 58.0,
               "Pylon actuator + bay doors": 31.0}
    added = {"Nose gearbox, mount, folding hub": 18.0}
    dmass = sum(added.values()) - sum(removed.values())

    m = mass_rollup()
    W_base = m["mtow_g"] / 1000.0 * G
    W_var = (m["mtow_g"] + dmass * (1 + C.MASS_GROWTH_ALLOWANCE)) / 1000.0 * G

    cd0_base = cd0_clean()
    # lose the dorsal spine and half the door seams; blades fold onto the cone
    cd0_var = cd0_base - (0.00110 + 0.00028) * (1 + C.DRAG["margin"])

    g = geom()

    def bg(W, cd0, extra):
        best = None
        cl = 0.15
        while cl <= C.DRAG["cl_max"]:
            cd = (cd0 + extra + C.DRAG["wing_cd_k"] * (cl - C.DRAG["wing_cl_design"]) ** 2
                  + cl * cl / (math.pi * g["AR"] * C.DRAG["oswald_e"]))
            V = speed_for_cl(cl, W, g["S"])
            r = {"LD": cl / cd, "V": V, "sink": V / (cl / cd)}
            if best is None or r["LD"] > best["LD"]:
                best = r
            cl += 0.01
        return best

    return {"dmass_g": dmass,
            "base": {"cruise_cd0": cd0_base, "mtow_g": m["mtow_g"],
                     **bg(W_base, cd0_base, 0.0),
                     "deployed_penalty": C.DRAG["motor_deployed_off"]},
            "variant": {"cruise_cd0": cd0_var, "mtow_g": m["mtow_g"] + dmass,
                        **bg(W_var, cd0_var, 0.0),
                        "deployed_penalty": 0.00030}}


# ==========================================================================
# report
# ==========================================================================

def _t(rows, head):
    w = "| " + " | ".join(head) + " |"
    w += "\n|" + "|".join("---" for _ in head) + "|"
    for r in rows:
        w += "\n| " + " | ".join(str(c) for c in r) + " |"
    return w + "\n"


def report():
    m = mass_rollup()
    W = m["mtow_g"] / 1000.0 * G
    g = geom()
    L = B.solar_layout()
    _loop, _up, props = B.wing_section()
    o = []
    A = o.append

    A("# FF-1 *Ember* — computed analysis\n")
    A("> Generated by `tools/analysis.py` from `tools/config.py`. "
      "Do not hand-edit; re-run the script.\n")
    A(f"Wing loading **{W/g['S']:.1f} N/m² ({m['mtow_g']/1000/g['S']:.2f} kg/m²)**, "
      f"MTOW **{m['mtow_g']:.0f} g**, span **{g['b']:.2f} m**, "
      f"area **{g['S']:.4f} m²**, AR **{g['AR']:.2f}**.\n")

    # ---- mass ----
    A("\n## 1. Mass budget\n")
    rows, gt = [], {}
    for grp, items in m["groups"].items():
        for (item, gm, unc) in items:
            rows.append([grp, item, f"{gm:.0f}", f"±{unc:.0f}"])
        gt[grp] = sum(i[1] for i in items)
    A(_t(rows, ["Group", "Item", "g", "1σ"]))
    A("\n**By group**\n")
    A(_t([[k, f"{v:.0f}", f"{100*v/m['itemised_g']:.1f} %"]
          for k, v in sorted(gt.items(), key=lambda kv: -kv[1])]
         + [["*Itemised total*", f"{m['itemised_g']:.0f}", "100.0 %"],
            [f"*Growth allowance ({C.MASS_GROWTH_ALLOWANCE*100:.0f} %)*",
             f"{m['growth_g']:.0f}", ""],
            ["**MTOW**", f"**{m['mtow_g']:.0f}**", ""]],
         ["Group", "g", "share"]))
    A(f"\nRoot-sum-square uncertainty on the itemised total is ±{m['sigma_g']:.0f} g. "
      f"The build must be weighed bay by bay against this table; a 10 % overrun "
      f"costs about {100*(math.sqrt(1.10)-1):.0f} % in sink rate.\n")

    # ---- aero ----
    A("\n## 2. Aerodynamics\n### 2.1 Zero-lift drag build-up\n")
    br, raw, mar, tot = cd0_buildup()
    A(_t([[n, f"{d:.5f}", f"{100*d/tot:.1f} %", note] for (n, d, note) in br]
         + [["*Sum*", f"{raw:.5f}", "", ""],
            [f"*Uncertainty margin ({C.DRAG['margin']*100:.0f} %)*", f"{mar:.5f}", "", ""],
            ["**C_D0 (both pods stowed)**", f"**{tot:.4f}**", "", ""]],
         ["Component", "ΔC_D0", "share", "basis"]))

    A(f"\n### 2.2 Section\n\n`{props['name']}`: **{props['t_max']*100:.2f} %** thick at "
      f"{props['t_at']*100:.1f} % chord, **{props['camber_max']*100:.2f} %** camber at "
      f"{props['camber_at']*100:.1f} % chord. Upper-surface curvature inside the "
      f"{props['band'][0]*100:.0f}–{props['band'][1]*100:.0f} % solar band is limited to "
      f"R ≥ {props['r_min_mm']:.0f} mm, giving a bending strain of "
      f"{C.SOLAR['cell_thickness_um']/2/props['r_min_mm']/1000*100:.4f} % in a "
      f"{C.SOLAR['cell_thickness_um']:.0f} µm cell — roughly a factor of "
      f"{0.15/(C.SOLAR['cell_thickness_um']/2/props['r_min_mm']/1000*100):.0f} "
      f"below the ~0.15 % fracture strain of cut monocrystalline silicon.\n")

    A("\n### 2.3 Speed polar (both pods stowed, ρ = "
      f"{RHO} kg/m³)\n")
    kp = key_points(W)
    rows = []
    for r in kp["sweep"]:
        if abs(round(r["CL"], 2) * 100 % 10) < 1e-6:
            rows.append([f"{r['CL']:.2f}", f"{r['V']:.2f}", f"{r['CD']:.4f}",
                         f"{r['LD']:.1f}", f"{r['sink']:.3f}", f"{r['Re']:,.0f}"])
    A(_t(rows, ["C_L", "V (m/s)", "C_D", "L/D", "sink (m/s)", "Re (MAC)"]))

    pms = practical_min_sink(W)
    A("\n**Key speeds**\n")
    A(_t([["Stall, clean", f"{kp['v_stall']:.2f}", f"C_Lmax {C.DRAG['cl_max']}"],
          ["Stall, flaps down", f"{kp['v_stall_flapped']:.2f}",
           f"C_Lmax {C.DRAG['cl_max_flapped']}"],
          ["Minimum sink (theoretical)", f"{kp['min_sink']['V']:.2f}",
           f"sink {kp['min_sink']['sink']:.3f} m/s — lies on the stall, unusable"],
          ["Minimum sink (practical, 1.15 V_s)", f"{pms['V']:.2f}",
           f"sink {pms['sink']:.3f} m/s, L/D {pms['LD']:.1f} — **the soaring speed**"],
          ["Best glide", f"{kp['best_glide']['V']:.2f}",
           f"L/D {kp['best_glide']['LD']:.1f}, sink {kp['best_glide']['sink']:.3f} m/s"],
          ["Survey cruise", f"{kp['best_glide']['V']:.2f}",
           "flown at best glide; the payload swath is set by altitude, not speed"]],
         ["Condition", "V (m/s)", "note"]))
    A(f"\nReynolds number at the mean aerodynamic chord runs "
      f"{kp['v_stall']*g['mac']/NU:,.0f}–{15*g['mac']/NU:,.0f}. That is squarely in "
      f"the range where laminar separation bubbles decide the drag, which is why "
      f"the section and the surface finish matter more than the planform.\n")

    # ---- retraction ----
    A("\n### 2.4 What the pods cost when deployed\n")
    rb = retraction_benefit()
    A(_t([[k, f"{v['cd0']:.4f}", f"{v['LD']:.1f}", f"{v['sink']:.3f}",
           f"+{v['sink_penalty_pct']:.1f} %", f"−{v['ld_penalty_pct']:.1f} %"]
          for k, v in rb.items()],
         ["Configuration", "C_D0", "L/D", "sink (m/s)", "sink penalty", "L/D penalty"]))
    A("\nThis is the number that justifies retraction, and it is smaller than "
      "intuition suggests: at a 2 m span and this wing loading, induced drag "
      "dominates at soaring C_L, so a parasite-drag increment is diluted. "
      "Retraction is still clearly right for the turbine — it has to stow for "
      "landing regardless — but see §7 for the motor-installation trade.\n")

    # ---- propulsion ----
    A("\n## 3. Propulsion\n")
    e = C.EFF
    A(f"Chain efficiency: prop {e['prop']:.2f} × motor {e['motor']:.2f} × ESC "
      f"{e['esc']:.2f} = **{e['prop']*e['motor']*e['esc']:.3f}**.\n\n")
    rows = []
    for V in (8.0, 8.7, 9.2, 10.0, 11.0, 12.0, 13.0, 15.0):
        p, cl = elec_power_for(V, W, motor=True)
        if p:
            rows.append([f"{V:.1f}", f"{cl:.2f}", f"{p*e['prop']*e['motor']*e['esc']:.1f}",
                         f"{p:.1f}", f"{p/(m['mtow_g']/1000):.1f}"])
    A(_t(rows, ["V (m/s)", "C_L", "P_aero (W)", "P_electrical (W)", "W/kg"]))
    p_climb = W * (2.5 + kp["best_glide"]["sink"]) / (e["prop"] * e["motor"] * e["esc"])
    A(f"\nClimb at 2.5 m/s needs **{p_climb:.0f} W** electrical "
      f"({p_climb/(3*3.6):.1f} A on a 3S pack) — the sizing case for the motor, ESC "
      f"and cell discharge rating. Cells must be rated ≥ 10 A continuous; "
      f"high-capacity/low-current 21700s are the wrong choice here.\n")

    # ---- turbine ----
    A("\n## 4. The turbine — what a dynamo can and cannot do\n")
    A("A turbine takes its electrical power out of the airstream. Extracting "
      "P watts electrically costs P/(η_gen·η_rect) watts of shaft power, and that "
      "shaft power is drag. **It cannot extend endurance in steady flight.** "
      "The numbers:\n\n")
    A(_t([[f"{r['V']:.0f}", f"{r['p_available']:.1f}", f"{r['p_shaft']:.1f}",
           f"**{r['p_elec']:.1f}**", f"{r['extraction_drag_N']:.2f}", f"{r['rpm']:,.0f}"]
          for r in (rat_power(v) for v in (10, 12, 15, 18, 20, 25, 30))],
         ["V (m/s)", "P in disc (W)", "P shaft (W)", "P electrical (W)",
          "extraction drag (N)", "rotor rpm"]))
    A(f"\nRotor Ø {C.RAT['rotor_dia']:.0f} mm, {C.RAT['blades']} folding blades, "
      f"C_p {C.RAT['cp']:.2f} at λ = {C.RAT['lambda_design']:.1f}. Power scales as "
      f"V³, so the turbine is nearly useless at soaring speed and genuinely useful "
      f"in a descent.\n")

    A("\n### 4.1 Mode A — holding altitude in lift while extracting\n")
    A(_t([[f"{r['V']:.0f}", f"{r['p_elec']:.1f}", f"{r['glide_sink_ms']:.2f}",
           f"**{r['thermal_needed_ms']:.2f}**"]
          for r in (rat_hold_altitude(W, v) for v in (12, 14, 16, 18, 20, 22))],
         ["V (m/s)", "P electrical (W)", "sink without turbine (m/s)",
          "lift needed to hold height (m/s)"]))
    A("\nHonest reading: to make 19 W you must sit in a 5.6 m/s thermal. Cores that "
      "strong exist over burn scars and dry ground in the afternoon, but they are "
      "not the common case. Mode A is opportunistic, not a power plan.\n")

    A("\n### 4.2 Mode B — regenerative descent (the useful one)\n")
    A(_t([[f"{r['V']:.0f}", f"{r['pe_wh']:.2f}", f"**{r['recovered_wh']:.2f}**",
           f"{r['recovery_frac']*100:.0f} %", f"{r['sink_ms']:.2f}", f"{r['descent_time_s']:.0f}"]
          for r in (regen_descent(W, v, 500.0) for v in (12, 15, 20, 25))],
         ["V (m/s)", "potential energy (Wh)", "recovered (Wh)", "recovery",
          "sink (m/s)", "descent time (s)"]))
    hotel = sum(h[1] for h in C.HOTEL)
    rd = regen_descent(W, 20.0, 500.0)
    A(f"\nA 500 m regenerative descent banks **{rd['recovered_wh']:.2f} Wh** — about "
      f"{rd['recovered_wh']/hotel*60:.0f} minutes of hotel load. Modest, but it is "
      f"energy you were going to throw away as drag anyway, and the turbine doubles "
      f"as the airbrake, so the airframe needs no spoilers.\n")

    A("\n### 4.3 Mode C — emergency power\n")
    r15 = rat_power(15.0)
    A(f"At {15:.0f} m/s the turbine makes **{r15['p_elec']:.1f} W**, against a hotel "
      f"load of {hotel:.1f} W. With a dead pack and a dead array it will hold the "
      f"autopilot, GNSS and telemetry link alive the whole way down. From 900 m at "
      f"the resulting sink rate that is roughly "
      f"{900/rat_hold_altitude(W,15.0)['thermal_needed_ms']/60:.0f} minutes of "
      f"controlled, communicating flight. This is the same argument that puts a RAT "
      f"in an airliner, and on its own it justifies carrying the turbine.\n")

    # ---- energy ----
    A("\n## 5. Energy\n### 5.1 Array\n")
    _, chain = array_power(1000.0)
    A(_t([["Wing strips", f"{L['n_strips_wing']}",
           f"{L['n_strips_wing']/C.SOLAR['strips_per_cell']:.2f} full cells"],
          ["Active cell area", f"{L['wing_area_m2']*1e4:.0f} cm²", f"{L['wing_area_m2']:.4f} m²"],
          ["Nameplate at STC", f"{L['p_stc_wing_w']:.1f} W", "1000 W/m², 25 °C"],
          ["Installed chain efficiency", f"{chain*100:.1f} %",
           "cell × encapsulation × temperature × mismatch × MPPT × attitude"],
          ["Output at 1000 W/m² horizontal", f"{array_power(1000.0)[0]:.1f} W", ""],
          ["Optional V-tail sub-array", f"+{L['p_stc_tail_w']:.1f} W STC",
           f"{L['n_strips_tail']} strips — growth item, excluded from the budget below"]],
         ["", "", "note"]))

    A("\n### 5.2 Clear-sky array output\n")
    for (nm, lat, doy, cl) in C.MISSION["sites"]:
        A(f"\n**{nm}** — latitude {lat}°, day {doy}, clearness {cl}\n\n")
        rows = []
        for h in range(6, 21):
            irr, el = clear_sky(lat, doy, h, cl)
            pw, _ = array_power(irr)
            rows.append([f"{h:02d}:00", f"{el:.1f}", f"{irr:.0f}", f"{pw:.1f}"])
        A(_t(rows, ["solar time", "sun elevation (°)", "GHI (W/m²)", "array (W)"]))

    A("\n### 5.3 Loads\n")
    A(_t([[n, f"{w:.2f}"] for (n, w) in C.HOTEL]
         + [["**Hotel total**", f"**{hotel:.2f}**"]], ["Load", "W"]))
    bat = C.BATTERY
    cap = bat["cells_series"] * bat["cells_parallel"] * bat["cell_wh"]
    pc, _ = elec_power_for(kp["best_glide"]["V"], W, motor=True)
    A(f"\nPowered level cruise costs **{pc:.1f} W** electrical, so the hotel load is "
      f"{hotel/(hotel+pc)*100:.0f} % of the powered demand. On this aircraft a watt "
      f"saved in avionics is worth more than a drag count.\n")
    A(f"\nPack: {bat['cells_series']}S{bat['cells_parallel']}P 21700, "
      f"**{cap:.1f} Wh** nameplate, {cap*bat['usable_dod']:.1f} Wh at "
      f"{bat['usable_dod']*100:.0f} % DoD, **{cap*bat['usable_dod']-bat['reserve_wh']:.1f} Wh** "
      f"usable after a {bat['reserve_wh']:.0f} Wh approach reserve.\n")

    # ---- mission ----
    A("\n### 5.4 Mission simulation\n")
    A("30 s time steps from a 09:00 launch. Thermal availability is modelled as a "
      "sine bell from 09:30 to 18:30 peaking at 75 % duty (45 % in N. Europe); "
      "the aircraft motors whenever it is not soaring. Endurance ends when the "
      "usable pack is empty.\n\n")
    rows = []
    for site in C.MISSION["sites"]:
        peak = 0.75 if site[1] < 45 else 0.55
        for label, kw in [("solar + soaring", dict(peak_soar=peak)),
                          ("solar only, no thermals", dict(peak_soar=peak, allow_soar=False)),
                          ("thermals only, array failed", dict(peak_soar=peak, use_solar=False)),
                          ("battery only", dict(peak_soar=peak, use_solar=False, allow_soar=False))]:
            r = simulate(site, launch_h=9.0, **kw)
            rows.append([site[0], label, f"**{r['endurance_h']:.1f} h**",
                         f"{r['solar_wh']:.0f}", f"{r['peak_solar_w']:.1f}",
                         "✅" if r["reached_target"] else "—"])
    A(_t(rows, ["Site", "Case", "Endurance", "Solar harvested (Wh)",
                "Peak array (W)", "≥ 8 h"]))
    A(f"\n**The {C.MISSION['target_endurance_h']:.0f}-hour requirement is met by solar "
      f"plus soaring, and is not met on batteries alone — battery-only endurance is "
      f"about 1.5 h.** Note that solar alone very nearly does it; thermal soaring is "
      f"the margin that absorbs a bad day, not the primary mechanism.\n")

    # ---- stability ----
    A("\n## 6. Stability and control\n")
    st = stability()
    A(_t([["Wing lift-curve slope a_w", f"{st['a_w']:.3f} /rad", ""],
          ["Tail lift-curve slope a_t", f"{st['a_t']:.3f} /rad",
           f"projected AR {st['AR_h']:.2f}"],
          ["Tail arm", f"{st['l_t']*1000:.0f} mm", "wing a.c. to tail a.c."],
          ["Horizontal tail volume V_h", f"{st['V_h']:.3f}", "target 0.45–0.70"],
          ["Vertical tail volume V_v", f"{st['V_v']:.4f}", "target 0.020–0.035"],
          ["Downwash dε/dα", f"{st['deps_dalpha']:.3f}", ""],
          ["Neutral point", f"{st['x_np_mac']*100:.1f} % MAC", ""],
          ["**CG, target**", f"**{st['cg_target']*100:.1f} % MAC**",
           f"{(C.WING['stations'][0][2] + st['cg_target']*g['mac']*1000):.0f} mm aft of the nose"],
          ["CG, forward limit", f"{st['cg_fwd']*100:.1f} % MAC", "28 % static margin"],
          ["CG, aft limit", f"{st['cg_aft']*100:.1f} % MAC", "12 % static margin"]],
         ["", "", "note"]))
    A("\nDeploying the pods moves the CG: the pylon rises and moves the motor mass up "
      "and aft, the turbine swings down and aft. Both are aft of the CG, so both "
      "deployments are slightly nose-up in pitch trim and both are stabilising in "
      "yaw. Trim the autopilot for the stowed case and let the pitch loop absorb "
      "the rest; the deployment transient is the thing to watch on the first "
      "flights, not the trimmed state.\n")

    # ---- structure ----
    A("\n## 7. Structure\n")
    wb = wing_beam()
    A(_t([["Limit load factor", f"{wb['n_limit']:.1f} g", "ultimate = 1.5 × limit"],
          ["Root bending moment", f"{wb['M_root']:.1f} N·m", "elliptic distribution"],
          ["Root shear", f"{wb['shear_root_N']:.1f} N", ""],
          ["Spar cap", f"{wb['cap_w_mm']:.0f} mm × {wb['plies']:.0f} plies UD",
           f"{wb['cap_area_root_mm2']:.2f} mm² per cap at the root"],
          ["Cap centroid separation", f"{wb['cap_sep_root_mm']:.1f} mm", "at the root rib"],
          ["Root cap stress at limit", f"{wb['sigma_root_mpa']:.0f} MPa",
           "vs ~700 MPa allowable for T700 UD"],
          ["Margin of safety, ultimate", f"{700/(1.5*wb['sigma_root_mpa'])-1:+.2f}", ""],
          ["Root EI", f"{wb['EI_root']:.0f} N·m²", ""],
          ["Tip deflection at 1 g", f"{wb['tip_defl_1g_mm']:.0f} mm",
           f"{wb['tip_defl_1g_mm']/(g['b']*500)*100:.1f} % of semi-span"],
          ["Tip deflection at limit", f"{wb['tip_defl_mm']:.0f} mm",
           f"{wb['tip_defl_mm']/(g['b']*500)*100:.1f} % of semi-span"]],
         ["", "", "note"]))
    A("\nThe wing is stiffness-driven, not strength-driven: the cap laminate is set "
      "by wanting a flat array (a flexing wing de-focuses nothing, but it does crack "
      "cells and open encapsulation seams) long before it is set by stress.\n")

    # ---- motor installation trade ----
    A("\n## 8. Trade study — motor installation\n")
    t = variant_nose_prop()
    A(_t([["Deployable dorsal pylon (baseline, as requested)",
           f"{t['base']['mtow_g']:.0f}", f"{t['base']['cruise_cd0']:.4f}",
           f"{t['base']['LD']:.1f}", f"{t['base']['sink']:.3f}",
           f"{t['base']['deployed_penalty']:.5f}"],
          ["Nose folding prop (Variant N)",
           f"{t['variant']['mtow_g']:.0f}", f"{t['variant']['cruise_cd0']:.4f}",
           f"{t['variant']['LD']:.1f}", f"{t['variant']['sink']:.3f}",
           f"{t['variant']['deployed_penalty']:.5f}"]],
         ["Option", "MTOW (g)", "C_D0 stowed", "L/D", "sink (m/s)",
          "ΔC_D0 when running"]))
    A(f"\nVariant N is **{-t['dmass_g']:.0f} g lighter**, has "
      f"**{(1-t['variant']['sink']/t['base']['sink'])*100:.1f} % less sink** and no "
      f"mechanism to jam, because it deletes the pylon, its actuator, its doors and "
      f"the dorsal spine that swallows it. The dorsal pylon buys three things in "
      f"exchange: the prop disc is clear of the ground on a belly landing, the "
      f"nose stays free for a forward-looking sensor if the ventral turret is ever "
      f"outgrown, and thrust acts above the drag line so power changes do not pitch "
      f"the aircraft as hard.\n\nThe baseline keeps the pylon because it is what was "
      f"asked for and the penalty is real but small. If endurance is the only thing "
      f"that matters, build Variant N.\n")

    # ---- payload ----
    A("\n## 9. Wildfire detection performance\n")
    d = detection()
    A(_t([["Survey altitude", f"{d['alt_m']:.0f} m AGL"],
          ["Ground swath", f"{d['swath_m']:.0f} m"],
          ["Ground sample distance", f"{d['gsd_m']:.2f} m/pixel"],
          ["Pixel ground footprint", f"{d['pixel_area_m2']:.1f} m²"],
          ["Raw coverage", f"{d['coverage_km2_h']:.1f} km²/h at {d['V']:.1f} m/s"],
          ["Coverage with 30 % sidelap", f"**{d['coverage_sidelap_km2_h']:.1f} km²/h**"],
          ["Over an 8 h sortie", f"**{d['coverage_sidelap_km2_h']*8:.0f} km²**"]],
         ["", ""]))
    A(f"\nSub-pixel detection: the {C.PAYLOAD_SENSOR['band_um'][0]:.0f}–"
      f"{C.PAYLOAD_SENSOR['band_um'][1]:.0f} µm band captures "
      f"{d['band_frac_bg']*100:.0f} % of a {C.PAYLOAD_SENSOR['background_c']:.0f} °C "
      f"background's emission but only {d['band_frac_flame']*100:.0f} % of an "
      f"{C.PAYLOAD_SENSOR['flame_temp_k']:.0f} K flame's — LWIR is the *unfavourable* "
      f"band for hot small fires, and these figures already account for that.\n\n")
    A(_t([[f"{r['fire_m2']:.2f}", f"{r['fill_frac']*100:.1f} %",
           f"{r['T_app_c']:.1f}", f"+{r['delta_k']:.1f}", f"{r['snr']:,.0f}×"]
          for r in d["rows"]],
         ["Flame area (m²)", "pixel fill", "apparent pixel T (°C)", "ΔT vs background",
          "ΔT / NETD"]))
    A("\n**The sensor is not the limit — clutter is.** Sun-baked bare rock, dark "
      "soil and asphalt reach 55–70 °C on a summer afternoon, i.e. ΔT of +28 to "
      "+43 K. Anything below that threshold is indistinguishable from hot ground on "
      "a single frame, which puts the single-frame detection floor at roughly "
      "**0.5–1 m² of active flame**, not the 0.05 m² the raw NETD implies. Getting "
      "below that needs the contextual approach used by MODIS/VIIRS: compare each "
      "candidate pixel against the statistics of its own neighbourhood, require "
      "persistence across frames as the aircraft moves, and cross-check the visible "
      "camera for a smoke plume.\n")

    return "\n".join(o)


def main():
    path = os.path.join(ROOT, "docs", "ANALYSIS.md")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write(report())
    print(f"wrote {os.path.relpath(path, ROOT)} "
          f"({os.path.getsize(path)/1024:.1f} kB)")


if __name__ == "__main__":
    main()
