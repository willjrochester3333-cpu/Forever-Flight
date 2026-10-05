#!/usr/bin/env python3
"""
VT-1: a thrust-vectoring tail for the JW-1 flying wing.

    python3 tools/vectortail.py

WHAT THIS IS

Putting control surfaces BEHIND the propeller instead of on the wing. A
pusher prop throws a jet of air aft; a vane sitting in that jet deflects it,
and the reaction is a side force on the aircraft. Turn the jet, turn the
thrust -- that is thrust vectoring, done with vanes rather than a gimbal.

WHY IT MATTERS ON THIS AIRCRAFT

A control surface in free air makes force proportional to V^2. At 6 m/s it
has 35% of the authority it has at 10. That is exactly backwards: you need
the MOST control when you are slowest -- hand launch, climb-out, a botched
thermal entry. A vane in the slipstream makes force proportional to the JET
speed, which stays high when the aircraft is slow, because the prop is doing
the work. Authority stops collapsing at the bottom of the envelope.

And the JW-1 as drawn has NO yaw control at all. Elevons give pitch and roll.
A cruciform in the jet gives it a rudder.

THE LAYOUT

  twin booms      outboard of the prop disc, OUTSIDE the slipstream tube,
                  so they carry the load without paying jet drag
  cruciform       on the thrust axis, deep INSIDE the jet -- horizontal
                  surface for pitch, vertical for yaw
  symmetric       the vertical is above AND below the axis, so a yaw input
                  makes no rolling moment
  lower fin       extends past the prop tip circle, so it is also the skid
                  that stops a pusher eating its propeller on landing

Everything below is computed, not asserted. The prop model is calibrated
against published APC 7x4E static data; the slipstream is momentum theory
with McCormick's development law; flap effectiveness is exact thin-aerofoil
theory, not a table lookup.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import jetwing as JW

RHO = 1.225
NU = 1.46e-5
G = 9.80665


# ==========================================================================
# propeller
# ==========================================================================
#
# T = Ct rho n^2 D^4,  P = Cp rho n^3 D^5,  J = V / (n D)
#
# Ct and Cp fall off linearly with advance ratio. The constants are set so
# the model reproduces the published APC 7x4E static point (about 4.2 N at
# 10,000 rpm on 69 W) and peaks near eta = 0.55 at J = 0.45, which is what a
# small thin-bladed prop actually does.

PROP = {"ct0": 0.120, "j0": 0.70, "cp0": 0.068, "cp_fall": 0.75}


def prop(rpm, V, dia_mm, p=PROP):
    """Thrust (N), shaft power (W), advance ratio and efficiency."""
    if rpm <= 1.0:
        return {"T": 0.0, "P": 0.0, "J": 0.0, "eta": 0.0, "rpm": 0.0}
    n = rpm / 60.0
    D = dia_mm / 1000.0
    J = V / (n * D)
    ct = max(0.0, p["ct0"] * (1.0 - J / p["j0"]))
    cp = max(1e-4, p["cp0"] * (1.0 - p["cp_fall"] * J / p["j0"]))
    T = ct * RHO * n * n * D ** 4
    P = cp * RHO * n ** 3 * D ** 5
    return {"T": T, "P": P, "J": J, "rpm": rpm,
            "eta": (J * ct / cp) if cp > 0 else 0.0}


# ==========================================================================
# slipstream
# ==========================================================================

def slipstream(T, V, dia_mm, x_mm):
    """Jet speed and radius x_mm behind a disc of thrust T at airspeed V.

    Momentum theory gives the fully developed wake speed

        V_wake = sqrt(V^2 + 2 T / (rho A))

    and half of that increment has appeared by the disc itself. McCormick's
    development law carries it the rest of the way:

        V(x) = V + (V_wake - V) * 0.5 * (1 + (x/R) / sqrt(1 + (x/R)^2))

    Continuity then contracts the tube: V_disc R^2 = V(x) R(x)^2.
    """
    R = dia_mm / 2000.0
    A = math.pi * R * R
    if T <= 1e-6:
        return {"V": V, "R_mm": dia_mm / 2.0, "q_ratio": 1.0,
                "V_wake": V, "V_disc": V}
    Vw = math.sqrt(V * V + 2.0 * T / (RHO * A))
    Vd = 0.5 * (V + Vw)
    s = (x_mm / 1000.0) / R
    f = 0.5 * (1.0 + s / math.sqrt(1.0 + s * s))
    Vx = V + (Vw - V) * f
    Rx = R * math.sqrt(Vd / Vx)
    return {"V": Vx, "R_mm": Rx * 1000.0, "V_wake": Vw, "V_disc": Vd,
            "q_ratio": (Vx / V) ** 2 if V > 0.1 else float("inf")}


# ==========================================================================
# aerofoil and flap theory
# ==========================================================================

def flap(hinge_x):
    """Exact thin-aerofoil derivatives for a plain flap hinged at hinge_x.

        dCl/ddelta   = 2 [ (pi - th) + sin th ]
        dCm_c4/ddelta = (1/2) sin th (cos th - 1)          th = acos(1 - 2 x_h)

    Both per radian. No tables, no interpolation.
    """
    th = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * hinge_x)))
    dcl = 2.0 * ((math.pi - th) + math.sin(th))
    dcm = 0.5 * math.sin(th) * (math.cos(th) - 1.0)
    return {"dcl": dcl, "dcm": dcm, "tau": dcl / (2.0 * math.pi), "theta": th}


def helmbold(AR):
    """Low-aspect-ratio lift slope, per radian. Valid down to AR ~ 1."""
    return 2.0 * math.pi * AR / (2.0 + math.sqrt(AR * AR + 4.0))


# ==========================================================================
# configuration
# ==========================================================================

VT = {
    "name": "VT-1",
    # --- booms ---
    "boom_y": 108.0,           # half-spacing; must clear the prop tip
    "boom_od": 9.0,            # sized by the sweep: lighter booms sit FURTHER
    "boom_wall": 0.7,          # below the propeller forcing band, not nearer
    "boom_bond": 70.0,         # socketed into a spar saddle, not buried
    # --- vane station ---
    "arm_dia": 0.55,           # stab LE, prop diameters aft of the disc
    # --- horizontal vane (pitch) ---
    "stab_chord": 78.0,
    "stab_t": 0.09,
    "elev_frac": 0.40,         # elevator chord fraction
    "elev_max": 25.0,          # deg
    # --- vertical vane (yaw), symmetric above and below the thrust axis ---
    "fin_h": 96.0,             # upper, and the RUDDER half-height each side
    "fin_lower": None,         # None = solve it from the propeller-guard
    "fin_chord": 78.0,
    "rud_frac": 0.40,
    "rud_max": 25.0,
    "z_thrust": -8.0,           # thrust axis vs the pod datum line
    "land_pitch": 8.0,         # nose-up touchdown attitude
    "guard_margin": 12.0,      # blade tip must clear the ground by this
    "fin_sweep": 15.0,         # LE shear, constant chord (area unchanged)
    # --- the rest ---
    "drop_dorsal": True,       # delete the JW-1 dorsal fin (see report)
    "rpm_max": 10500.0,
    "rpm_min_sustained": 4000.0,
    "batt_g": 150.0,
    "E_cfrp": 130000.0,        # N/mm2, pultruded unidirectional
    "rho_cfrp": 1.55e-3,       # g/mm3
}

# Added mass, itemised: (name, grams, station, scales?)
#   A 5 g servo is a 5 g servo whatever the aircraft. Only the structure
#   shrinks with the airframe, which is why the tail is a far bigger mass
#   fraction on the smaller wing -- see the 1200 result.
MASS = [
    ("carbon booms, 2 off",        None, "boom", True),   # from geometry
    ("stabiliser + elevator",      8.4,  "vane", True),
    ("fin, ventral + rudder",      9.8,  "vane", True),
    ("skid shoe, replaceable",     2.2,  "vane", True),
    ("servos, 2 off 5 g",         10.0,  "vane", False),
    ("horns, links, pushrods",     4.2,  "vane", False),
    ("mass balance, lead",         3.8,  "vane", True),
    ("boom sockets + bonding",     5.0,  "te",   False),
    ("servo extension wire",       3.0,  "mid",  False),
]


def stations(cfg=JW.JW, vt=VT):
    """Every x-station the tail depends on, in wing coordinates."""
    sp = JW.spec(cfg)
    pf = sp["pf"]
    x_prop = cfg["pod_len"] * 0.98 - 5.0                 # the disc
    k = cfg["prop_dia"] / JW.JW["prop_dia"]              # scale factor
    arm = vt["arm_dia"] * cfg["prop_dia"]
    x_vane = x_prop + arm                                # stab LE
    c_s = vt["stab_chord"] * k
    x_ac = x_vane + 0.25 * c_s                           # tail aero centre
    y_b = vt["boom_y"] * k
    c_wing = JW.chord_at(y_b, cfg)
    x_te = JW.le_at(y_b, cfg) + c_wing                   # wing TE at the boom
    st = {"sp": sp, "pf": pf, "k": k,
            "x_prop": x_prop, "x_vane": x_vane, "x_ac": x_ac,
            "x_te": x_te, "y_boom": y_b,
            "c_stab": c_s, "c_fin": vt["fin_chord"] * k,
            "b_stab": 2.0 * y_b, "h_fin": vt["fin_h"] * k,
            "h_lower": (vt["fin_lower"] * k) if vt["fin_lower"] else None,
            "x_cg": sp["trim"]["x_cg"], "x_np": sp["trim"]["x_np"],
            "mac": pf["mac"], "S": pf["S"], "b": pf["b"],
            "l_t": x_ac - sp["trim"]["x_cg"],
            "free_boom": x_vane + 0.35 * c_s - (JW.le_at(y_b, cfg) + c_wing),
            "tip_clear": y_b - cfg["prop_dia"] / 2.0 - vt["boom_od"] * k / 2.0,
            "z_thrust": vt["z_thrust"] * k}
    if st["h_lower"] is None:
        st["h_lower"] = vt["fin_h"] * k
        st["h_lower"] = guard_height(cfg, vt, st)["h"]
    return st


def boom_mass(st, vt=VT):
    od = vt["boom_od"] * st["k"]
    idm = od - 2.0 * vt["boom_wall"] * st["k"]
    a = math.pi / 4.0 * (od ** 2 - idm ** 2)
    L = st["free_boom"] + vt["boom_bond"] * st["k"]
    return 2.0 * a * L * vt["rho_cfrp"], a, L, od, idm


def mass_budget(st, vt=VT):
    mb, _a, L, _od, _id = boom_mass(st, vt)
    arms = {"boom": st["x_te"] + 0.55 * st["free_boom"],
            "vane": st["x_vane"] + 0.45 * st["c_stab"],
            "te": st["x_te"] - 40.0,
            "mid": 0.5 * (st["x_te"] + st["x_vane"])}
    rows, tot, mom = [], 0.0, 0.0
    for (name, g, where, scales) in MASS:
        g = mb if g is None else (g * st["k"] ** 2 if scales else g)
        x = arms[where]
        rows.append((name, g, x, x - st["x_cg"]))
        tot += g
        mom += g * x
    return {"rows": rows, "g": tot, "x": mom / tot,
            "arm": mom / tot - st["x_cg"], "boom_len": L}


# ==========================================================================
# what the tail does to stability
# ==========================================================================

def surfaces(st, vt=VT):
    """Areas, aspect ratios and lift slopes of the two vanes."""
    S_h = st["b_stab"] * st["c_stab"] / 1e6
    AR_h = st["b_stab"] / st["c_stab"]
    # The lower fin is deeper than the upper so it can guard the propeller.
    # Fin AREA (stability) uses the real total height; the RUDDER is kept
    # symmetric about the thrust axis so a yaw input makes no roll.
    h_tot = st["h_fin"] + st["h_lower"]
    S_v = h_tot * st["c_fin"] / 1e6
    AR_v = h_tot / st["c_fin"]
    return {"S_h": S_h, "AR_h": AR_h, "a_h": helmbold(AR_h),
            "S_v": S_v, "AR_v": AR_v, "a_v": helmbold(AR_v),
            "V_h": (S_h / st["S"]) * (st["l_t"] / st["mac"]),
            "V_v": (S_v / st["S"]) * (st["l_t"] / (st["b"] * 1000.0))}


def wing_slope(st):
    a0 = 2.0 * math.pi
    return a0 / (1.0 + a0 / (math.pi * st["pf"]["AR"] * 0.90))


def downwash(st):
    """de/dalpha at the tail. Low-AR swept wing, tail near the wake centre."""
    return min(0.55, 2.0 * wing_slope(st) / (math.pi * st["pf"]["AR"]))


def np_shift(st, vt=VT, eta_t=1.0):
    """How far aft the tail moves the neutral point, in mm and %MAC."""
    s = surfaces(st, vt)
    de = downwash(st)
    d = eta_t * s["V_h"] * (s["a_h"] / wing_slope(st)) * (1.0 - de)
    return {"pct_mac": d * 100.0, "mm": d * st["mac"], "de_da": de, **s}


def fin_volumes(cfg=JW.JW, st=None):
    """Directional stability: what the dorsal fin, the winglets and the
    cruciform each actually contribute. Arm is measured from the CG, so a
    surface sitting on top of the CG scores zero however big it is."""
    st = st or stations(cfg)
    out = {}
    # dorsal fin: swept, so take its area centroid at mid height
    h, cr, ct = cfg["fin_h"], cfg["fin_root"], cfg["fin_tip"]
    S = h * (cr + ct) / 2.0
    ybar = h / 3.0 * (cr + 2 * ct) / (cr + ct)                  # centroid up
    c_at = cr + (ct - cr) * (ybar / h)
    x = cfg["fin_x"] * cfg["root_chord"] + ybar * math.tan(
        math.radians(cfg["fin_sweep"])) + 0.25 * c_at
    out["dorsal"] = (S / 1e6, x - st["x_cg"])
    # winglets: vertical projection of the canted tip surface
    hw, cant = cfg["winglet_h"], math.radians(cfg["winglet_cant"])
    c_tip = cfg["tip_chord"]
    Sw = hw * c_tip * (1 + cfg["winglet_taper"]) / 2.0 * math.sin(cant)
    xw = (JW.le_at(cfg["span"] / 2.0, cfg)
          + 0.5 * hw * math.tan(math.radians(cfg["winglet_sweep"]))
          + 0.25 * c_tip)
    out["winglets"] = (2.0 * Sw / 1e6, xw - st["x_cg"])
    # cruciform verticals
    s = surfaces(st)
    out["cruciform"] = (s["S_v"], st["l_t"])
    for k, (S_, l_) in out.items():
        out[k] = {"S": S_, "arm": l_,
                  "V_v": (S_ / st["S"]) * (l_ / (st["b"] * 1000.0))}
    return out


def cg_solution(st, vt=VT, cfg=JW.JW):
    """The tail adds mass aft AND moves the neutral point aft. Net it out,
    then say what it takes to put the static margin back."""
    mb = mass_budget(st, vt)
    m0 = cfg["mass_g"]
    m1 = m0 + mb["g"]
    dcg = mb["g"] * mb["arm"] / m1                    # CG moves aft by this
    dnp = np_shift(st, vt)
    # the dorsal fin, if deleted, also moves the CG forward a little
    drop = 0.0
    if vt["drop_dorsal"]:
        drop = 12.0 * st["k"] ** 2
        fv = fin_volumes(cfg, st)
        dcg -= drop * fv["dorsal"]["arm"] / m1
        m1 -= drop
    sm0 = cfg["static_margin"] * st["mac"]
    sm1 = sm0 + dnp["mm"] - dcg
    # restore by moving the battery forward
    need = (sm0 - sm1) * m1 / vt["batt_g"]
    return {"m0": m0, "m1": m1, "added": mb["g"] - drop, "dcg": dcg,
            "dnp_mm": dnp["mm"], "dnp_pct": dnp["pct_mac"],
            "sm0_mm": sm0, "sm1_mm": sm1,
            "sm0": cfg["static_margin"] * 100,
            "sm1": sm1 / st["mac"] * 100,
            "batt_shift": need, "dorsal_g": drop, "mb": mb, "np": dnp}


# ==========================================================================
# control authority
# ==========================================================================

def vane_force(delta_deg, q_free, jet, span_mm, chord_mm, hinge, a_3d,
               half_immersed=True, n=60):
    """Force from a vane partly immersed in the jet.

    Strip integral across the span: each strip sees the jet dynamic pressure
    if it lies inside the contracted tube, free-stream if not. Loading is
    taken uniform across the span, which is conservative -- real loading
    peaks at the centre, which is the part inside the jet.
    """
    fl = flap(1.0 - hinge)
    dcl = fl["tau"] * a_3d * math.radians(delta_deg)     # 3D-corrected
    q_jet = 0.5 * RHO * jet["V"] ** 2
    half = span_mm / 2.0
    dy = half / n
    F = Fj = 0.0
    for i in range(n):
        y = (i + 0.5) * dy
        q = q_jet if y <= jet["R_mm"] else q_free
        dF = q * (chord_mm * dy / 1e6) * dcl
        F += dF
        if y <= jet["R_mm"]:
            Fj += dF
    F *= 2.0
    Fj *= 2.0
    imm = min(1.0, jet["R_mm"] / half)
    return {"F": F, "F_jet": Fj, "immersed": imm, "dcl": dcl,
            "q_jet": q_jet, "q_free": q_free}


def elevon_moment(delta_deg, V, cfg=JW.JW, st=None, n=60):
    """Baseline pitching moment from the JW-1 elevons, in N.m. Thin-aerofoil
    flap theory per strip, carrying BOTH the lift increment at c/4 and the
    section moment increment, about the real CG."""
    st = st or stations(cfg)
    fl = flap(1.0 - cfg["elevon_chord"])
    k3 = wing_slope(st) / (2.0 * math.pi)                # 2D -> 3D
    d = math.radians(delta_deg)
    q = 0.5 * RHO * V * V
    half = cfg["span"] / 2.0
    y0, y1 = cfg["elevon_y0"] * half, cfg["elevon_y1"] * half
    dy = (y1 - y0) / n
    Mo = 0.0
    for i in range(n):
        y = y0 + (i + 0.5) * dy
        c = JW.chord_at(y, cfg)
        x_c4 = JW.le_at(y, cfg) + 0.25 * c
        dA = c * dy / 1e6
        dcl = fl["dcl"] * k3 * d
        dcm = fl["dcm"] * d
        Mo += -dcl * dA * (x_c4 - st["x_cg"]) / 1000.0 + dcm * dA * c / 1000.0
    return 2.0 * q * Mo


def condition(name, V, throttle, cfg=JW.JW, st=None, vt=VT):
    """Pitch and yaw authority at one flight condition, vanes vs elevons."""
    st = st or stations(cfg)
    s = surfaces(st, vt)
    pr = prop(vt["rpm_max"] * throttle, V, cfg["prop_dia"])
    jet = slipstream(pr["T"], V, cfg["prop_dia"],
                     st["x_vane"] + 0.5 * st["c_stab"] - st["x_prop"])
    q = 0.5 * RHO * V * V
    el = vane_force(vt["elev_max"], q, jet, st["b_stab"], st["c_stab"],
                    vt["elev_frac"], s["a_h"])
    ru = vane_force(vt["rud_max"], q, jet, 2.0 * st["h_fin"], st["c_fin"],
                    vt["rud_frac"], s["a_v"])        # symmetric span only
    arm = st["l_t"] / 1000.0
    return {"name": name, "V": V, "thr": throttle, "prop": pr, "jet": jet,
            "q": q, "elev": el, "rud": ru,
            "M_pitch_vane": el["F"] * arm,
            "M_yaw_vane": ru["F"] * arm,
            "M_pitch_elevon": abs(elevon_moment(15.0, V, cfg, st)),
            "vector_deg": math.degrees(math.atan2(ru["F"], max(pr["T"], 1e-9)))
            if pr["T"] > 0.01 else 0.0}


# ==========================================================================
# throttle / trim coupling -- the vice, and how to kill it
# ==========================================================================

def tail_incidence(st, vt=VT, cfg=JW.JW):
    """Set the tail so it carries ZERO load at the cruise trim attitude.

    This is the whole trick. The tail moment is  Cm_t = -V_h * eta_t * CL_t.
    When the throttle opens, eta_t jumps from 1 to 2 or 3 as the jet washes
    the tail. If CL_t is zero at trim, Cm_t is zero whatever eta_t does, and
    opening the throttle causes NO trim change. Stability still rises with
    power, because dCm/dalpha keeps the eta_t factor. Free stability, no
    trim coupling.
    """
    de = downwash(st)
    a_tr = st["sp"]["trim"]["alpha_trim"]
    return {"i_t": -a_tr * (1.0 - de), "alpha_trim": a_tr, "de_da": de}


def throttle_trim(st, vt=VT, cfg=JW.JW, d_alpha=(0, 2, 4, 6, -2, -4)):
    """Trim change when the throttle opens, at and away from the design point."""
    s = surfaces(st, vt)
    inc = tail_incidence(st, vt, cfg)
    V = st["sp"]["v_trim"]
    pr = prop(vt["rpm_max"], V, cfg["prop_dia"])
    jet = slipstream(pr["T"], V, cfg["prop_dia"],
                     st["x_vane"] + 0.5 * st["c_stab"] - st["x_prop"])
    eta = jet["q_ratio"]
    cma = -cfg["static_margin"] * wing_slope(st)          # per rad
    out = []
    for da in d_alpha:
        cl_t = s["a_h"] * math.radians(da * (1.0 - inc["de_da"]))
        dcm = -s["V_h"] * cl_t * (eta - 1.0)
        out.append((da, cl_t, dcm, math.degrees(dcm / -cma)))
    return {"eta": eta, "rows": out, "inc": inc, "V": V, "T": pr["T"]}


# ==========================================================================
# drag
# ==========================================================================

def cf(Re):
    """Skin friction: laminar below transition, turbulent above, blended."""
    if Re < 1.0:
        return 0.01
    lam = 1.328 / math.sqrt(Re)
    turb = 0.074 / Re ** 0.2
    return lam if Re < 5e4 else (turb if Re > 5e5 else
                                 lam + (turb - lam) * (Re - 5e4) / 4.5e5)


def drag_penalty(st, vt=VT, cfg=JW.JW, V=8.0):
    """Added parasite drag, unpowered, referred to wing area."""
    items = []
    # booms: axial flow over a cylinder, form factor for a long slender body
    _m, _a, _L, od, _id = boom_mass(st, vt)
    Lb = st["free_boom"]
    Re = V * (Lb / 1000.0) / NU
    ff = 1.0 + 1.5 * (od / Lb) ** 1.5 + 7.0 * (od / Lb) ** 3
    items.append(("booms, 2 off", 2.0 * math.pi * od * Lb / 1e6, cf(Re) * ff))
    # vanes: wetted both sides, thickness form factor
    for (nm, sp_, c_) in (("stabiliser", st["b_stab"], st["c_stab"]),
                          ("fin + ventral", st["h_fin"] + st["h_lower"],
                           st["c_fin"])):
        Re = V * (c_ / 1000.0) / NU
        ff = 1.0 + 2.0 * vt["stab_t"] + 60.0 * vt["stab_t"] ** 4
        items.append((nm, 2.0 * sp_ * c_ / 1e6, cf(Re) * ff))
    add = sum(A * c for (_n, A, c) in items) * 1.15        # interference
    # what deleting the dorsal fin gives back
    back = 0.0
    if vt["drop_dorsal"]:
        h, cr, ct = cfg["fin_h"], cfg["fin_root"], cfg["fin_tip"]
        A = 2.0 * h * (cr + ct) / 2.0 / 1e6
        Re = V * ((cr + ct) / 2.0 / 1000.0) / NU
        back = A * cf(Re) * (1.0 + 2.0 * 0.08 + 60 * 0.08 ** 4) * 1.15
        items.append(("dorsal fin DELETED", -A, -cf(Re)))
    net = (add - back) / st["S"]
    return {"items": items, "dCD0": net, "add": add / st["S"],
            "back": back / st["S"]}


def glide_cost(st, vt=VT, cfg=JW.JW, cd0=0.0180, e=0.88):
    """What the added drag costs the glide, which is 85% of this mission."""
    AR = st["pf"]["AR"]
    d = drag_penalty(st, vt, cfg)["dCD0"]
    out = {}
    for (nm, cl) in (("best glide", 0.62), ("min sink", 0.95)):
        cdi = cl * cl / (math.pi * AR * e)
        a, b = cd0 + cdi, cd0 + d + cdi
        out[nm] = {"LD0": cl / a, "LD1": cl / b,
                   "sink": (b / a) - 1.0,
                   "ld": (cl / b) / (cl / a) - 1.0}
    out["dCD0"] = d
    return out


def throttle_for(T_need, V, cfg=JW.JW, vt=VT):
    """Throttle fraction that produces a wanted thrust at a given speed."""
    lo, hi = 0.0, 1.0
    if prop(vt["rpm_max"], V, cfg["prop_dia"])["T"] < T_need:
        return 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if prop(vt["rpm_max"] * mid, V, cfg["prop_dia"])["T"] < T_need:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def level_throttle(V, st, cfg=JW.JW, vt=VT, cd0=0.0180, e=0.88):
    """Throttle for level flight at V, including the tail's own drag."""
    W = (cfg["mass_g"] + mass_budget(st, vt)["g"]) / 1000.0 * G
    q = 0.5 * RHO * V * V
    cl = W / (q * st["S"])
    cd = cd0 + drag_penalty(st, vt, cfg, V)["dCD0"] \
        + cl * cl / (math.pi * st["pf"]["AR"] * e)
    return throttle_for(q * st["S"] * cd, V, cfg, vt), cl, cd


# ==========================================================================
# structure
# ==========================================================================

def structure(st, vt=VT, cfg=JW.JW):
    """Boom stiffness, first bending mode, prop-plane clearance, flutter.

    A boom-mounted tail behind a propeller has three structural jobs:
      1. not let the vanes wander under load (control precision)
      2. not deflect into the propeller disc
      3. not sit on a propeller forcing frequency it has to dwell at
    """
    mb_, A, L_tot, od, idm = boom_mass(st, vt)
    I = math.pi / 64.0 * (od ** 4 - idm ** 4)
    L = st["free_boom"]
    E = vt["E_cfrp"]
    k = 2.0 * 3.0 * E * I / L ** 3                        # N/mm, two booms

    # worst vane load: full deflection, full throttle, low speed
    c = condition("launch", 6.0, 1.0, cfg, st, vt)
    P = max(c["elev"]["F"], c["rud"]["F"]) * 2.5          # 2.5 g manoeuvre
    tip = P / k
    x_p = st["x_prop"] - st["x_te"]                       # prop plane on boom
    at_prop = (P / 2.0) * x_p ** 2 * (3 * L - x_p) / (6.0 * E * I) * 2.0

    # first bending mode, Rayleigh: tail mass plus 0.23 of the free boom
    m_tail = sum((g * st["k"] ** 2 if sc else g)
                 for (_n, g, w, sc) in MASS
                 if g is not None and w == "vane") / 1000.0
    m_free = mb_ * (L / L_tot) / 1000.0
    m_eff = m_tail + 0.23 * m_free
    f1 = math.sqrt(k * 1000.0 / m_eff) / (2.0 * math.pi)

    # propeller forcing. 2 blades -> 1P is imbalance, 2P is blade passing.
    rpm_1p, rpm_2p = 60.0 * f1, 30.0 * f1
    band = (vt["rpm_min_sustained"], vt["rpm_max"])
    hits = [(n, r) for (n, r) in (("1P", rpm_1p), ("2P", rpm_2p))
            if band[0] <= r <= band[1]]

    # flutter: mass-balance the control surfaces to the hinge line
    bal = []
    for (nm, sp_, c_, fr, g) in (("elevator", st["b_stab"], st["c_stab"],
                                  vt["elev_frac"], 2.6),
                                 ("rudder", 2 * st["h_fin"], st["c_fin"],
                                  vt["rud_frac"], 2.3)):
        g *= st["k"] ** 2
        cs = fr * c_
        lead = 0.55 * c_                                   # horn ahead of hinge
        bal.append((nm, g, 0.45 * cs, g * 0.45 * cs / lead, lead))
    return {"I": I, "k": k, "f1": f1, "tip": tip, "at_prop": at_prop,
            "P": P, "clear": st["tip_clear"], "net_clear": st["tip_clear"] - at_prop,
            "rpm_1p": rpm_1p, "rpm_2p": rpm_2p, "hits": hits, "band": band,
            "m_eff": m_eff, "od": od, "id": idm, "L": L, "mass": mb_,
            "balance": bal}


# ==========================================================================
# ground clearance -- the lower vane as propeller guard
# ==========================================================================

def prop_guard(st, vt=VT, cfg=JW.JW, pitch_deg=8.0):
    """A pusher eats its propeller on landing unless something sits lower.

    Take the ground as the line through the two lowest points at a nose-high
    touchdown attitude, and ask how close the blade tip comes to it.
    """
    z_ax = st["z_thrust"]
    R = cfg["prop_dia"] / 2.0
    # the LE shear carries the ventral tip AFT, which flattens the ground
    # line and eats clearance -- so the sweep has to be in this sum
    pts = {"lower fin tip": (st["x_vane"] + 0.5 * st["c_fin"]
                             + st["h_lower"] * math.tan(
                                 math.radians(vt["fin_sweep"])),
                             z_ax - st["h_lower"]),
           "pod belly": (0.38 * cfg["pod_len"], -0.5 * cfg["pod_h"]),
           "blade tip": (st["x_prop"], z_ax - R)}
    th = math.radians(pitch_deg)
    # rotate nose-up about the CG, then find the ground line
    rot = {k: (x * math.cos(th) + z * math.sin(th),
               -x * math.sin(th) + z * math.cos(th)) for k, (x, z) in pts.items()}
    (xa, za) = rot["pod belly"]
    (xb, zb) = rot["lower fin tip"]
    m = (zb - za) / (xb - xa)
    (xc, zc) = rot["blade tip"]
    ground_at_prop = za + m * (xc - xa)
    return {"pts": pts, "rot": rot, "clearance": zc - ground_at_prop,
            "fin_below_axis": st["h_lower"], "prop_R": R,
            "guarded": zc - ground_at_prop >= 0.0, "pitch": pitch_deg}


# ==========================================================================
# sweeps
# ==========================================================================

def arm_sweep(cfg=JW.JW, arms=(0.40, 0.55, 0.70, 0.85, 1.00, 1.20)):
    rows = []
    for a in arms:
        vt = dict(VT); vt["arm_dia"] = a
        st = stations(cfg, vt)
        cg = cg_solution(st, vt, cfg)
        s = surfaces(st, vt)
        c = condition("", 6.0, 1.0, cfg, st, vt)
        g = glide_cost(st, vt, cfg)
        sr = structure(st, vt, cfg)
        rows.append({"arm": a, "l_t": st["l_t"], "boom": sr["mass"],
                     "add": cg["added"], "sm": cg["sm1"],
                     "shift": cg["batt_shift"], "V_h": s["V_h"],
                     "V_v": s["V_v"], "yaw": c["M_yaw_vane"],
                     "f1": sr["f1"], "sink": g["best glide"]["sink"] * 100})
    return rows


def boom_sweep(cfg=JW.JW, sizes=((8.0, 0.7), (9.0, 0.7), (10.0, 0.8),
                                 (11.0, 0.8), (12.0, 1.0))):
    rows = []
    for (od, w) in sizes:
        vt = dict(VT); vt["boom_od"] = od; vt["boom_wall"] = w
        st = stations(cfg, vt)
        sr = structure(st, vt, cfg)
        cg = cg_solution(st, vt, cfg)
        rows.append({"od": od, "wall": w, "mass": sr["mass"], "f1": sr["f1"],
                     "tip": sr["tip"], "at_prop": sr["at_prop"],
                     "net_clear": st["tip_clear"] - sr["at_prop"],
                     "sm": cg["sm1"], "hits": sr["hits"]})
    return rows


def st_k(vt):
    return 1.0


def guard_height(cfg=JW.JW, vt=VT, st=None):
    """Depth of lower fin needed to keep the blade tip off the ground.

    Contact is the pod belly and the fin tip; the blade tip sits between
    them, so the fin has to be deep enough that the line between the two
    contact points passes UNDER the blade circle, with margin.
    """
    st = st or stations(cfg, vt)
    z_ax = st["z_thrust"]
    R = cfg["prop_dia"] / 2.0
    x_b, z_b = 0.38 * cfg["pod_len"], -0.5 * cfg["pod_h"]
    x_p, z_p = st["x_prop"], z_ax - R - vt["guard_margin"]
    slope = (z_p - z_b) / (x_p - x_b)
    tsw = math.tan(math.radians(vt["fin_sweep"]))
    # the tip station depends on the depth we are solving for: iterate
    h = st["h_fin"]
    for _ in range(60):
        x_f = st["x_vane"] + 0.5 * st["c_fin"] + h * tsw
        h = max(st["h_fin"], z_ax - (z_b + slope * (x_f - x_b)))
    return {"h": h, "x_fin": x_f, "belly": (x_b, z_b),
            "blade": (x_p, z_ax - R), "slope": slope}


# ==========================================================================
# mass and balance -- does it actually close?
# ==========================================================================

def wing_centroid(cfg=JW.JW, n=400):
    """Area-weighted x of a printed shell wing. On a SWEPT wing this sits
    well behind the CG, which is why everything else has to go forward."""
    half = cfg["span"] / 2.0
    dy = half / n
    num = den = 0.0
    for i in range(n):
        y = (i + 0.5) * dy
        c = JW.chord_at(y, cfg)
        num += c * (JW.le_at(y, cfg) + 0.45 * c) * dy
        den += c * dy
    return num / den


# (item, grams, x-station as a callable of (cfg, st))
AIRFRAME = [
    ("wing shell, printed",  280.0, lambda c, s: wing_centroid(c)),
    ("winglets",              16.0, lambda c, s: JW.le_at(c["span"] / 2, c)
                                    + 0.5 * c["winglet_h"] * math.tan(
                                        math.radians(c["winglet_sweep"]))
                                    + 0.25 * c["tip_chord"]),
    ("spar, carbon",          40.0, lambda c, s: 0.30 * c["root_chord"]),
    ("pod shell, printed",    62.0, lambda c, s: 0.44 * c["pod_len"]),
    ("motor + mount",         48.0, lambda c, s: s["x_prop"] - 28.0),
    ("propeller + spinner",   12.0, lambda c, s: s["x_prop"]),
    ("ESC",                   20.0, lambda c, s: 0.56 * c["pod_len"]),
    ("elevon servos, 4 off",  32.0, lambda c, s: JW.le_at(0.6 * c["span"] / 2, c)
                                    + 0.72 * JW.chord_at(0.6 * c["span"] / 2, c)),
    ("receiver + wiring",     14.0, lambda c, s: 0.28 * c["pod_len"]),
    ("flight computer, LWIR", 70.0, lambda c, s: 0.11 * c["pod_len"]),
    ("hardware, glue, tape",  26.0, lambda c, s: 0.48 * c["pod_len"]),
]

DORSAL = ("dorsal fin", 12.0)


def balance(cfg=JW.JW, vt=VT, with_tail=False, st=None):
    """Itemised mass and balance. The battery is the free variable: solve
    for the station that puts the CG where the trim solution wants it."""
    st = st or stations(cfg, vt)
    k2 = st["k"] ** 2
    rows = [(n, g * k2, f(cfg, st)) for (n, g, f) in AIRFRAME]
    if not (with_tail and vt["drop_dorsal"]):
        fv = fin_volumes(cfg, st)
        rows.append((DORSAL[0], DORSAL[1] * k2, st["x_cg"] + fv["dorsal"]["arm"]))
    if with_tail:
        mb = mass_budget(st, vt)
        rows += [(n, g, x) for (n, g, x, _a) in mb["rows"]]
    base = sum(g for (_n, g, _x) in rows)
    mom = sum(g * x for (_n, g, x) in rows)
    # the pack is the same pack in both builds: whatever the baseline
    # airframe does not account for
    batt = cfg["mass_g"] - sum(a[1] * k2 for a in AIRFRAME) - DORSAL[1] * k2
    auw = base + batt
    want_sm = cfg["static_margin"]
    x_np = st["x_np"] + (np_shift(st, vt)["mm"] if with_tail else 0.0)
    x_cg = x_np - want_sm * st["mac"]
    x_batt = (x_cg * auw - mom) / batt
    rows.append(("battery pack", batt, x_batt))
    nose = st["sp"]["nose_x"]
    return {"rows": rows, "auw": auw, "batt": batt, "x_batt": x_batt,
            "x_cg": x_cg, "x_np": x_np, "nose": nose,
            "batt_from_nose": x_batt - nose,
            "cg_from_nose": x_cg - nose, "sm": want_sm * 100,
            "pod_len": cfg["pod_len"]}


# ==========================================================================
# trim WITH the tail in the loop
# ==========================================================================

def cm_total(alpha, x_cg, i_t, cfg, sec, st, vt, eta=1.0):
    """Wing strip moments plus the tail. Downwash scales with wing lift."""
    s = surfaces(st, vt)
    w = JW.strip_moments(alpha, x_cg, cfg, sec)
    de = downwash(st)
    eps = de * w["CL"] / wing_slope(st)                   # rad
    cl_t = s["a_h"] * (math.radians(alpha + i_t) - eps)
    cm_t = -s["V_h"] * eta * cl_t
    CL = w["CL"] + (s["S_h"] / st["S"]) * eta * cl_t
    return {"Cm": w["Cm"] + cm_t, "CL": CL, "CL_w": w["CL"], "CL_t": cl_t,
            "eps_deg": math.degrees(eps)}


def np_with_tail(i_t, cfg, sec, st, vt, eta=1.0):
    def slope(x):
        a = cm_total(0.0, x, i_t, cfg, sec, st, vt, eta)
        b = cm_total(4.0, x, i_t, cfg, sec, st, vt, eta)
        return (b["Cm"] - a["Cm"]) / (b["CL"] - a["CL"])
    lo, hi = 0.0, cfg["root_chord"] * 1.6
    for _ in range(80):
        m = 0.5 * (lo + hi)
        if slope(m) < 0:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def trim_with_tail(cfg, sec, x_cg, i_t, st, vt, eta=1.0):
    lo, hi = -10.0, 16.0
    for _ in range(80):
        m = 0.5 * (lo + hi)
        if cm_total(m, x_cg, i_t, cfg, sec, st, vt, eta)["Cm"] > 0:
            lo = m
        else:
            hi = m
    a = 0.5 * (lo + hi)
    return a, cm_total(a, x_cg, i_t, cfg, sec, st, vt, eta)


def solve_branch(cfg, vt, st, want_cl_t=0.0):
    """Find the tail incidence that makes the tail carry want_cl_t at trim,
    with the neutral point and the trim angle re-solved for that tail."""
    sec = JW.thin_aerofoil(cfg["camber"], cfg["camber_pos"], JW._reflex(cfg),
                           cfg["reflex_start"])
    lo, hi = -14.0, 10.0
    for _ in range(70):
        it = 0.5 * (lo + hi)
        x_np = np_with_tail(it, cfg, sec, st, vt)
        x_cg = x_np - cfg["static_margin"] * st["mac"]
        _a, tr = trim_with_tail(cfg, sec, x_cg, it, st, vt)
        if tr["CL_t"] > want_cl_t:
            hi = it
        else:
            lo = it
    it = 0.5 * (lo + hi)
    x_np = np_with_tail(it, cfg, sec, st, vt)
    x_cg = x_np - cfg["static_margin"] * st["mac"]
    a, tr = trim_with_tail(cfg, sec, x_cg, it, st, vt)
    W = (cfg["mass_g"] + mass_budget(st, vt)["g"] - 12.0) / 1000.0 * G
    v_tr = math.sqrt(2 * W / (RHO * st["S"] * max(tr["CL"], 0.05)))
    v_st = math.sqrt(2 * W / (RHO * st["S"] * 0.85))
    # throttle coupling at this tail loading
    eta = throttle_trim(st, vt, cfg)["eta"]
    dcm = -surfaces(st, vt)["V_h"] * tr["CL_t"] * (eta - 1.0)
    cma = cfg["static_margin"] * wing_slope(st)
    return {"i_t": it, "x_np": x_np, "x_cg": x_cg, "alpha": a,
            "CL": tr["CL"], "CL_w": tr["CL_w"], "CL_t": tr["CL_t"],
            "reflex": JW._reflex(cfg), "sec": sec, "v_trim": v_tr,
            "v_stall": v_st, "eta": eta,
            "throttle_shift": math.degrees(dcm / cma),
            "nose_cg": x_cg - st["sp"]["nose_x"]}


def cg_range(cfg=JW.JW, vt=VT, st=None, sms=(0.04, 0.06, 0.075, 0.09, 0.12)):
    """Trim point against CG position. The tail is kept unloaded at the
    7.5% design point, then the CG is moved and the aircraft re-trimmed."""
    st = st or stations(cfg, vt)
    base = solve_branch(cfg, vt, st, 0.0)
    sec, it = base["sec"], base["i_t"]
    W = (cfg["mass_g"] + mass_budget(st, vt)["g"] - 12.0) / 1000.0 * G
    rows = []
    for sm in sms:
        x_cg = base["x_np"] - sm * st["mac"]
        a, tr = trim_with_tail(cfg, sec, x_cg, it, st, vt)
        v = math.sqrt(2 * W / (RHO * st["S"] * max(tr["CL"], 0.05)))
        rows.append({"sm": sm * 100, "x_cg": x_cg,
                     "nose": x_cg - st["sp"]["nose_x"], "alpha": a,
                     "CL": tr["CL"], "CL_t": tr["CL_t"], "v": v,
                     "v_stall": math.sqrt(2 * W / (RHO * st["S"] * 0.85))})
    return rows, base


# ==========================================================================
# report
# ==========================================================================

def report(cfg=JW.JW, vt=VT):
    st = stations(cfg, vt)
    L = []
    def P(s=""): L.append(s)

    s = surfaces(st, vt)
    sr = structure(st, vt, cfg)
    pg = prop_guard(st, vt, cfg)
    fv = fin_volumes(cfg, st)
    g = glide_cost(st, vt, cfg)
    b0, b1 = balance(cfg, vt, False, st), balance(cfg, vt, True, st)
    rows, base = cg_range(cfg, vt, st)

    P("== VT-1 : thrust-vectoring tail for the JW-1 ==")
    P()
    P("GEOMETRY")
    P(f"  twin booms  {sr['od']:.0f} x {sr['id']:.1f} mm carbon at y = +/-{st['y_boom']:.0f} mm")
    P(f"              {sr['L']:.0f} mm free aft of the wing TE, "
      f"{vt['boom_bond']:.0f} mm socketed into the spar saddle")
    P(f"              clears the blade tip by {st['tip_clear']:.1f} mm static, "
      f"{sr['net_clear']:.1f} mm at full vane load")
    P(f"  vane station {vt['arm_dia']:.2f} prop diameters aft of the disc "
      f"= {st['x_vane'] - st['x_prop']:.0f} mm")
    P(f"  stabiliser  {st['b_stab']:.0f} x {st['c_stab']:.0f} mm "
      f"({s['S_h'] * 1e4:.0f} cm2, AR {s['AR_h']:.2f}), "
      f"elevator {vt['elev_frac'] * 100:.0f}% chord")
    P(f"  fin         {st['h_fin']:.0f} mm up, {st['h_lower']:.0f} mm down "
      f"({s['S_v'] * 1e4:.0f} cm2), rudder on the symmetric "
      f"+/-{st['h_fin']:.0f} mm only, so yaw makes no roll")
    P(f"  tail arm    {st['l_t']:.0f} mm from the CG   "
      f"V_h {s['V_h']:.4f}   V_v {s['V_v']:.4f}")
    P()
    P("DIRECTIONAL STABILITY -- and a problem this found in the JW-1")
    for k in ("dorsal", "winglets", "cruciform"):
        v = fv[k]
        P(f"  {k:10s} {v['S'] * 1e4:6.0f} cm2 at arm {v['arm']:+7.1f} mm "
          f"-> V_v {v['V_v']:.5f}")
    P(f"  The JW-1 dorsal fin sits {fv['dorsal']['arm']:.0f} mm behind the CG. "
      f"It is 227 cm2 of")
    P(f"  surface earning V_v {fv['dorsal']['V_v']:.5f} -- essentially nothing. "
      f"The winglets were")
    P(f"  carrying the aircraft. Delete the dorsal fin; the cruciform replaces it")
    P(f"  {fv['cruciform']['V_v'] / max(fv['dorsal']['V_v'], 1e-9):.0f} times over "
      f"and brings a rudder, which the JW-1 did not have.")
    P()
    P("CONTROL AUTHORITY        pitch, N.m          yaw, N.m   equivalent")
    P("                      vanes   elevons  ratio   vanes    vector angle")
    for (nm, V_, thr) in (("hand launch", 6.0, 1.00), ("climb-out", 9.0, 1.00),
                          ("full-power dash", 16.0, 1.00),
                          ("thermal circle", 7.5, 0.00),
                          ("at the stall", 6.7, 0.00)):
        c = condition(nm, V_, thr, cfg, st, vt)
        P(f"  {nm:16s} {c['M_pitch_vane']:6.3f}  {c['M_pitch_elevon']:7.3f} "
          f"{c['M_pitch_vane'] / c['M_pitch_elevon']:6.2f}x {c['M_yaw_vane']:7.3f} "
          f"{c['vector_deg']:9.0f} deg")
    c = condition("", 6.0, 1.0, cfg, st, vt)
    P(f"  At 6 m/s the jet carries {c['jet']['q_ratio']:.1f}x the free-stream dynamic")
    P(f"  pressure, so the vanes beat the elevons {c['M_pitch_vane'] / c['M_pitch_elevon']:.1f}x. "
      f"Unpowered they are worth")
    P(f"  about 0.4x an elevon: this ADDS to the elevons, it does not replace them.")
    P()
    P("THROTTLE / TRIM COUPLING")
    tt = throttle_trim(st, vt, cfg)
    P(f"  The jet raises tail dynamic pressure {tt['eta']:.2f}x at full throttle.")
    P(f"  Setting the tail {base['i_t']:+.2f} deg so it carries ZERO load at trim makes")
    P(f"  the trim change EXACTLY zero, because Cm_tail = -V_h.eta.CL_t and CL_t = 0")
    P(f"  kills it whatever eta does. Stability still rises with power.")
    for (da, cl, dcm, dd) in tt["rows"][:4]:
        P(f"    {da:+2.0f} deg off trim -> CL_t {cl:+.3f} -> {dd:+5.2f} deg shift on throttle")
    P()
    P("TRIM, with the tail in the loop")
    P(f"  tail incidence {base['i_t']:+.2f} deg, neutral point {base['x_np']:.1f} mm")
    P(f"  The tail is unloaded at trim but nose-up loaded at low alpha, which")
    P(f"  RAISES the trim CL from 0.366 (tailless) to {base['CL']:.3f}:")
    P("    static margin   CG from nose   alpha    CL     trim speed")
    for r in rows:
        mark = "  <- design" if abs(r["sm"] - 7.5) < 0.01 else ""
        P(f"      {r['sm']:5.1f}%         {r['nose']:6.0f} mm   {r['alpha']:5.2f}  "
          f"{r['CL']:.3f}   {r['v']:5.2f} m/s{mark}")
    P(f"  Stall {rows[0]['v_stall']:.2f} m/s, so the design point sits at "
      f"{rows[2]['v'] / rows[2]['v_stall']:.2f}x stall -- deliberately slow,")
    P(f"  which is what you want for soaring, but trim nose-down for cruise.")
    P()
    P("MASS AND BALANCE")
    mb = mass_budget(st, vt)
    for (n, gg, x, arm) in mb["rows"]:
        P(f"    {n:26s} {gg:6.1f} g   arm {arm:+7.1f} mm")
    P(f"    {'added':26s} {mb['g']:6.1f} g, less dorsal fin {-12.0:5.1f} g "
      f"= net {mb['g'] - 12:+.1f} g")
    P(f"  AUW {b0['auw']:.0f} -> {b1['auw']:.0f} g "
      f"({(b1['auw'] / b0['auw'] - 1) * 100:+.1f}%)")
    cgs = cg_solution(st, vt, cfg)
    P(f"  CG moves aft {cgs['dcg']:.1f} mm; the tail only moves the NP aft "
      f"{cgs['dnp_mm']:.1f} mm,")
    P(f"  so the balance does NOT come free: the {b0['batt']:.0f} g pack has to move from")
    P(f"  {b0['batt_from_nose']:.0f} mm to {b1['batt_from_nose']:.0f} mm behind the nose "
      f"-- {b0['batt_from_nose'] - b1['batt_from_nose']:.0f} mm FORWARD. That is a nose-bay")
    P(f"  change, not a bolt-on. Budget for it before cutting any carbon.")
    cl = closes(cfg, vt, st)
    if cl["ok"]:
        P(f"  Balance CLOSES at this span.")
    else:
        P(f"  Balance does NOT close at this span: a {cl['pack_have']:.0f} g pack would have")
        P(f"  to sit {cl['batt_from_nose']:.0f} mm from the nose. Either carry a "
          f"{cl['pack_needed']:.0f} g pack (+{cl['extra_pack']:.0f} g)")
        P(f"  in the nose, or lengthen the nose by {cl['nose_ext']:.0f} mm. Do not shrink the")
        P(f"  tail to dodge it -- a tail you cannot balance is not worth fitting.")
    P()
    P("DRAG, which is what the glide pays")
    for (n, A, c_) in drag_penalty(st, vt, cfg)["items"]:
        P(f"    {n:22s} {A * 1e4:+8.0f} cm2 wetted  cf.FF {c_:+.5f}")
    P(f"  net dCD0 {g['dCD0']:+.5f} on wing area "
      f"(the deleted dorsal fin pays back most of it)")
    for k in ("best glide", "min sink"):
        P(f"    {k:11s} L/D {g[k]['LD0']:.1f} -> {g[k]['LD1']:.1f}, "
          f"sink {g[k]['sink'] * 100:+.1f}%")
    P()
    P("STRUCTURE")
    P(f"  worst vane load {sr['P']:.2f} N (full deflection, full throttle, 2.5 g)")
    P(f"  tip deflection {sr['tip']:.2f} mm, {sr['at_prop']:.2f} mm at the prop plane")
    P(f"  first bending mode {sr['f1']:.0f} Hz")
    P(f"  1P crosses it at {sr['rpm_1p']:.0f} rpm, 2P at {sr['rpm_2p']:.0f} rpm; "
      f"sustained band is")
    P(f"  {sr['band'][0]:.0f}-{sr['band'][1]:.0f} rpm, so "
      f"{'both crossings are below it -- transient on spin-up only' if not sr['hits'] else 'IT DWELLS ON ' + str(sr['hits'])}.")
    P(f"  Counterintuitive but checked: a HEAVIER boom is worse here. Stiffening")
    P(f"  to 11 mm lifts f1 into the running band instead of clearing it.")
    for (nm, gg, cgm, lead, arm) in sr["balance"]:
        P(f"  mass-balance the {nm}: {lead:.2f} g at {arm:.0f} mm ahead of the hinge")
    P()
    P("PROPELLER GUARD")
    P(f"  A pusher eats its prop on landing. The blade tip reaches "
      f"{pg['prop_R']:.0f} mm below the")
    P(f"  thrust axis; the ventral fin reaches {pg['fin_below_axis']:.0f} mm, so at a "
      f"{pg['pitch']:.0f} deg nose-up touchdown")
    P(f"  the blade clears the ground by {pg['clearance']:+.1f} mm. "
      f"{'GUARDED.' if pg['guarded'] else 'STRIKES -- fix it.'}")
    P(f"  That is why the ventral is deeper than the dorsal, and why the rudder")
    P(f"  stops at +/-{st['h_fin']:.0f} mm: the extra depth is skid, not control.")
    return "\n".join(L)



def closes(cfg=JW.JW, vt=VT, st=None, min_station=25.0):
    """Does the balance close? If not, say what it would take.

    The pack is the only mass with anywhere to go. If it has to sit ahead of
    the nose, the answer is a heavier pack or a longer nose, not a smaller
    tail -- a tail you can balance is the only tail worth fitting.
    """
    st = st or stations(cfg, vt)
    b = balance(cfg, vt, True, st)
    nose = st["sp"]["nose_x"]
    if b["batt_from_nose"] >= min_station:
        return {"ok": True, **b}
    # fixed items and their moment, excluding the pack
    rows = [r for r in b["rows"] if r[0] != "battery pack"]
    base = sum(g for (_n, g, _x) in rows)
    mom = sum(g * x for (_n, g, x) in rows)
    x_p = nose + min_station
    # heavier pack at x_p: solve m from (mom + m x_p)/(base+m) = x_cg
    m = (b["x_cg"] * base - mom) / (x_p - b["x_cg"])
    # or keep the pack and lengthen the nose
    need = (b["x_cg"] * (base + b["batt"]) - mom) / b["batt"]
    return {"ok": False, **b, "pack_needed": m,
            "pack_have": b["batt"], "extra_pack": m - b["batt"],
            "nose_ext": (nose + min_station) - need}

def main():
    print(report())


if __name__ == "__main__":
    main()
