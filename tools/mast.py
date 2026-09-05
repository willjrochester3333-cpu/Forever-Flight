#!/usr/bin/env python3
"""
VDM-1: vertical deployment mast for the FF-1 energy-recovery turbine.

Sizes a telescoping vertical mast driven by one micro linear actuator, as an
alternative to the baseline swing arm. Solves, in order:

  geometry  -> required stroke, from rotor radius and body clearance
  stages    -> minimum stage count that nests inside the fuselage depth
  drive     -> actuator stroke and force through the reduction chain
  structure -> bending deflection, joint lash, first bending frequency
  mass/perf -> what it costs the aircraft

Run:  python3 tools/mast.py
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G = C.MISSION["g"]

# --------------------------------------------------------------------------
# design inputs
# --------------------------------------------------------------------------

STAGES = 3

MAST = {
    "station_x": 425.0,        # just aft of the wing root TE bulkhead
    "rotor_dia": 150.0,        # unchanged -- see the rotor trade below
    "nacelle_r": 15.0,
    "stowed_env_r": 23.0,      # nacelle + blades folded back along it
    "clearance_frac": 0.20,    # disc top clear of the fairing, x rotor dia
    "overlap_frac": 0.36,      # retained tube overlap at full extension
    "tube_gap": 1.5,           # per-side nesting clearance, mm
    "deck_margin": 0.0,        # keep the nested stack below the deck
    "mu_bush": 0.12,           # PTFE-lined bushing on hard-anodised alloy
    "eta_chain": 0.85,         # rack-pinion doubler chain efficiency
    "eta_bellcrank": 0.92,
    "v_retract": 20.0,         # retract airspeed limit, m/s
    "n_manoeuvre": 3.0,        # retract under this load factor
    "stiction_factor": 1.5,
    "E_carbon": 90.0e9,        # square braided/pultruded tube
    "rho_carbon": 1550.0,
    "wall": 0.8,               # mm
    # Rectangular section: the air load is fore-aft, so the section is deep in
    # X and narrow in Y. Deep-in-X also gives the racks somewhere to live and
    # reacts the generator's torque without a keyway.
    "trunk_x": 34.0,           # fore-aft, mm
    "trunk_y": 22.0,           # lateral, mm
    "nest_gap": 1.0,           # per-side sliding clearance
}

# Representative micro linear actuators. VERIFY against the current datasheet
# before ordering -- these are class figures, not a price list.
ACTUATORS = [
    # name,            stroke, force N, speed mm/s, mass g, retracted body mm
    ("PQ12-P 100:1",     20.0,   50.0,   6.0,  15.0,  44.0),
    ("L12-30 100:1",     30.0,   45.0,  13.0,  34.0,  81.0),
    ("L12-30 210:1",     30.0,   80.0,   6.0,  34.0,  81.0),
    ("L12-50 100:1",     50.0,   45.0,  13.0,  40.0, 101.0),
    ("L12-50 210:1",     50.0,   80.0,   6.0,  40.0, 101.0),
    ("L12-100 210:1",   100.0,   80.0,   6.0,  56.0, 151.0),
    ("L16-50 63:1",      50.0,  150.0,  13.0,  84.0, 128.0),
]

MAST_MASS = [
    ("Fixed trunk, bushings, bulkhead doublers", 16.0),
    ("Stage 1 tube, yoke head, rack",            12.0),
    ("Stage 2 tube, racks, pinion carrier",      11.0),
    ("Stage 3 tube, nacelle mount, rack",         9.0),
    ("Pinions, bushings, hardware",              10.0),
    ("Micro linear actuator + mount",            46.0),
    ("Bellcrank, con-rod, pivots",                9.0),
    ("Bay doors, cam, wiper seal",               14.0),
    ("End sensors + magnets",                     4.0),
    ("Wiring, connector",                         5.0),
]
SWING_ARM_MASS = 38.0 + 8.0     # arm/latch/doors + its servo


# --------------------------------------------------------------------------
# fuselage section at the mast station
# --------------------------------------------------------------------------

def fuse_at(x):
    s = C.FUSELAGE["stations"]
    for i in range(len(s) - 1):
        x0, w0, h0, c0 = s[i]
        x1, w1, h1, c1 = s[i + 1]
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            w = w0 + t * (w1 - w0)
            h = h0 + t * (h1 - h0)
            c = c0 + t * (c1 - c0)
            return {"w": w, "h": h, "deck": c + h / 2, "keel": c - h / 2}
    x0, w0, h0, c0 = s[-1]
    return {"w": w0, "h": h0, "deck": c0 + h0 / 2, "keel": c0 - h0 / 2}


# --------------------------------------------------------------------------
# 1. geometry -- what stroke the rotor actually demands
# --------------------------------------------------------------------------

def geometry(m=MAST, n_stages=None):
    n_stages = n_stages or STAGES
    f = fuse_at(m["station_x"])
    R = m["rotor_dia"] / 2.0
    clear = m["clearance_frac"] * m["rotor_dia"]

    e = None
    for _ in range(60):                       # stroke and stage length co-depend
        stage_len = (e or 45.0) / (1.0 - m["overlap_frac"])
        nested = stage_len + 4.0              # end fittings

        # nested stack hangs from the deck; the nacelle bolts to its bottom
        stack_top = f["deck"] - m["deck_margin"]
        stack_bot = stack_top - nested
        z_stow = stack_bot - m["nacelle_r"]
        env_bot = z_stow - m["stowed_env_r"]
        fairing_depth = max(0.0, f["keel"] - env_bot)

        z_dep = env_bot - clear - R
        stroke = z_stow - z_dep
        e_new = stroke / n_stages
        if e and abs(e_new - e) < 1e-6:
            break
        e = e_new

    return {"fuse": f, "rotor_R": R, "clearance": clear,
            "stage_travel": e, "stage_len": stage_len, "nested": nested,
            "stack_top": stack_top, "stack_bot": stack_bot,
            "z_stow": z_stow, "z_deployed": z_dep,
            "env_bot": env_bot, "fairing_depth": fairing_depth,
            "stroke": stroke, "n_stages": n_stages,
            "disc_top": z_dep + R, "disc_bot": z_dep - R,
            "below_keel": f["keel"] - (z_dep - R)}


def min_stages(m=MAST, limit=8):
    """Fewest stages whose nested stack fits under the deck."""
    out = []
    for n in range(1, limit + 1):
        g = geometry(m, n)
        f = g["fuse"]
        fits = g["nested"] <= f["h"] + 1e-9
        out.append({"n": n, "stroke": g["stroke"], "stage_travel": g["stage_travel"],
                    "nested": g["nested"], "fits": fits,
                    "fairing_depth": g["fairing_depth"], "avail": f["h"]})
    return out


# --------------------------------------------------------------------------
# 2. drive -- friction, load, actuator input
# --------------------------------------------------------------------------

def tube_sizes(m=MAST, n_stages=None):
    """(x, y) outside dimensions, trunk first then each moving stage."""
    n_stages = n_stages or STAGES
    step = 2.0 * (m["wall"] + m["nest_gap"])
    return [(m["trunk_x"] - i * step, m["trunk_y"] - i * step)
            for i in range(n_stages + 1)]


def section_I(ax, ay, t):
    """Second moment about the lateral axis, for a fore-aft bending load."""
    return (ax ** 3 * ay - (ax - 2 * t) ** 3 * (ay - 2 * t)) / 12.0


def tube_mass_g(ax, ay, L, m=MAST):
    t = m["wall"]
    a = (ax * ay - (ax - 2 * t) * (ay - 2 * t)) * 1e-6   # m^2
    return m["rho_carbon"] * a * (L / 1000.0) * 1000.0


def moving_mass_g(n_stages=None, m=MAST):
    n_stages = n_stages or STAGES
    g = geometry(m, n_stages)
    tubes = sum(tube_mass_g(ax, ay, g["stage_len"], m)
                for (ax, ay) in tube_sizes(m, n_stages)[1:])
    fittings = 5.0 * n_stages          # bushings, racks, pinion carriers
    return 18.0 + 44.0 + 12.0 + tubes + fittings   # rotor + gen + nacelle


def drag_on_deployed(m=MAST, g=None):
    """Air load on the extended mast + braked rotor, at the retract speed."""
    g = g or geometry(m)
    q = 0.5 * C.MISSION["rho_cruise"] * m["v_retract"] ** 2
    blade_area = C.RAT["blades"] * (g["rotor_R"] - C.RAT["hub_r"]) \
        * C.RAT["blade_chord"] * 0.78 / 1e6
    d_rotor = q * blade_area * 1.20                     # stopped blades, bluff
    d_nac = q * math.pi * (m["nacelle_r"] / 1000.0) ** 2 * 0.40
    exposed = (g["z_stow"] - g["z_deployed"]) / 1000.0
    d_mast = q * exposed * (m["trunk_y"] * 0.90 / 1000.0) * 0.55   # faired
    return {"q": q, "rotor": d_rotor, "nacelle": d_nac, "mast": d_mast,
            "total": d_rotor + d_nac + d_mast,
            "arm_below_trunk": (g["fuse"]["keel"] - g["z_deployed"]) / 1000.0}


def drive(m=MAST, n_stages=None):
    n_stages = n_stages or STAGES
    g = geometry(m, n_stages)
    d = drag_on_deployed(m, g)
    D = d["total"]

    # Each joint reacts the moment from everything below it as a bushing couple.
    overlap = g["stage_len"] * m["overlap_frac"] / 1000.0
    joints = []
    for j in range(n_stages):
        # joint j sits this far above the drag centroid when fully extended
        arm = d["arm_below_trunk"] - j * g["stage_travel"] / 1000.0
        arm = max(arm, 0.010)
        M = D * arm
        R_couple = M / overlap
        f = m["mu_bush"] * 2.0 * R_couple
        joints.append({"j": j + 1, "arm_m": arm, "moment": M,
                       "bush_N": R_couple, "friction_N": f})
    fric = sum(j["friction_N"] for j in joints)

    mm = moving_mass_g(n_stages, m) / 1000.0
    weight = mm * G * m["n_manoeuvre"]

    load = (fric + weight) * m["stiction_factor"]
    f_in = n_stages * load / (m["eta_chain"] * m["eta_bellcrank"])
    return {"geom": g, "drag": d, "joints": joints, "friction_N": fric,
            "moving_mass_g": mm * 1000, "weight_N": weight,
            "design_load_N": load, "actuator_force_N": f_in,
            "actuator_stroke_mm": g["stage_travel"],
            "overlap_mm": overlap * 1000}


def pick_actuator(dr):
    need_s, need_f = dr["actuator_stroke_mm"], dr["actuator_force_N"]
    rows = []
    for (name, s, f, v, mass, body) in ACTUATORS:
        rows.append({"name": name, "stroke": s, "force": f, "speed": v,
                     "mass": mass, "body": body,
                     "stroke_ok": s >= need_s,
                     "margin": f / need_f,
                     "deploy_s": dr["actuator_stroke_mm"] / v,
                     "ok": s >= need_s and f / need_f >= 2.0 and mass <= 60})
    return rows


# --------------------------------------------------------------------------
# 3. structure -- deflection, lash, resonance
# --------------------------------------------------------------------------

def structure(m=MAST, n_stages=3, lash_per_side=0.04):
    g = geometry(m, n_stages)
    ods = tube_sizes(m, n_stages)
    E, w = m["E_carbon"], m["wall"]

    # extended mast: segments from the trunk bottom down to the nacelle
    keel = g["fuse"]["keel"]
    seg_len = (keel - g["z_deployed"]) / n_stages / 1000.0
    segs = [{"od": ods[i + 1],
             "EI": E * section_I(ods[i + 1][0], ods[i + 1][1], w) * 1e-12,
             "L": seg_len}
            for i in range(n_stages)]

    # unit tip load: integrate M/EI from the tip up
    N = 400
    total_L = seg_len * n_stages
    dx = total_L / N
    theta = defl = 0.0
    for i in range(N):
        s = (i + 0.5) * dx                       # distance from the tip
        k = min(int(s / seg_len), n_stages - 1)
        theta += (1.0 * s) / segs[k]["EI"] * dx  # M = 1 N * s
        defl += theta * dx
    k_tip = 1.0 / defl                           # N/m

    m_tip = moving_mass_g(n_stages, m) / 1000.0
    m_tube = sum(tube_mass_g(s["od"][0], s["od"][1], s["L"] * 1000, m) / 1000.0
                 for s in segs)
    f1 = (1.0 / (2 * math.pi)) * math.sqrt(k_tip / (m_tip + 0.24 * m_tube))

    # rotational lash at each joint, accumulated to the tip
    overlap = g["stage_len"] * m["overlap_frac"]
    lash = 0.0
    for j in range(n_stages):
        below = (keel - g["z_deployed"]) - j * seg_len * 1000.0
        lash += (2 * lash_per_side / overlap) * below

    d = drag_on_deployed(m, g)
    v_max = 25.0                      # top of the turbine operating band
    rpm = C.RAT["lambda_design"] * v_max / (g["rotor_R"] / 1000.0) * 60 / (2 * math.pi)
    return {"segments": segs, "k_tip_N_m": k_tip,
            "defl_mm": d["total"] / k_tip * 1000,
            "lash_mm": lash, "f1_hz": f1, "mast_mass_g": m_tube * 1000,
            "rotor_1p_hz": rpm / 60.0, "rotor_rpm": rpm,
            "freq_ratio": f1 / (rpm / 60.0), "ods": ods}


# --------------------------------------------------------------------------
# 4. what it costs the aircraft
# --------------------------------------------------------------------------

def aircraft_effect(n_stages=None):
    n_stages = n_stages or STAGES
    import analysis as AN          # lazy: analysis imports build_model
    base = AN.mass_rollup()
    dm = sum(g for (_n, g) in MAST_MASS) - SWING_ARM_MASS
    # config already carries the mast masses, so the swing-arm aircraft is the
    # lighter one; compare like for like.
    mtow1 = base["mtow_g"]
    mtow0 = mtow1 - dm * (1 + C.MASS_GROWTH_ALLOWANCE)
    gm = AN.geom()
    W0, W1 = mtow0 / 1000 * G, mtow1 / 1000 * G

    def bg(W, dcd0=0.0):
        best = None
        cl = 0.15
        while cl <= C.DRAG["cl_max"]:
            cd = AN.polar(cl) + dcd0
            V = AN.speed_for_cl(cl, W, gm["S"])
            r = {"LD": cl / cd, "V": V, "sink": V / (cl / cd)}
            if best is None or r["LD"] > best["LD"]:
                best = r
            cl += 0.005
        return best

    # deeper ventral fairing for the nested stack
    g3 = geometry(MAST, n_stages)
    d_fair = g3["fairing_depth"]
    base_fair = 30.0
    dcd0 = max(0.0, (d_fair - base_fair) / base_fair) * 0.00075 * \
        (1 + C.DRAG["margin"])

    a, b = bg(W0), bg(W1, dcd0)

    # CG travel on deployment: swing arm moves the pod aft, the mast does not
    pod_g = 18.0 + 44.0 + 26.0 + 38.0
    rat = C.RAT
    x_st = rat["pivot"][0] + rat["arm_len"] * math.cos(math.radians(rat["stowed_angle"]))
    x_dp = rat["pivot"][0] + rat["arm_len"] * math.cos(math.radians(rat["deployed_angle"]))
    dx_swing = pod_g * (x_dp - x_st) / mtow0
    return {"dmass_g": dm, "mtow0": mtow0, "mtow1": mtow1,
            "dcd0_fairing": dcd0, "fairing_depth": d_fair,
            "base": a, "mast": b,
            "sink_pct": (b["sink"] / a["sink"] - 1) * 100,
            "cg_shift_swing_mm": dx_swing,
            "cg_shift_swing_mac": dx_swing / (gm["mac"] * 1000) * 100,
            "cg_shift_mast_mm": 0.0}


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------

def _t(rows, head):
    w = "| " + " | ".join(head) + " |"
    w += "\n|" + "|".join("---" for _ in head) + "|"
    for r in rows:
        w += "\n| " + " | ".join(str(c) for c in r) + " |"
    return w + "\n"


def report():
    g, dr, st = geometry(), drive(), structure()
    ae = aircraft_effect()
    f = g["fuse"]
    o, A = [], None
    A = o.append

    A("# VDM-1 — vertical deployment mast\n")
    A("> Generated by `tools/mast.py`. Re-run after any change to "
      "`tools/config.py`.\n")
    A(f"A telescoping vertical mast that lowers the energy-recovery turbine "
      f"**{g['stroke']:.0f} mm** straight down, driven by one micro linear "
      f"actuator moving **{g['stage_travel']:.1f} mm**. It replaces the "
      f"baseline swing arm (see [DESIGN.md](DESIGN.md) §6).\n")

    A("\n## 1. Why bother — the swing arm's real problem\n")
    swing_travel = abs(C.RAT["arm_len"] * (math.cos(math.radians(C.RAT["deployed_angle"]))
                                           - math.cos(math.radians(C.RAT["stowed_angle"]))))
    A(f"A pivoting arm carries the turbine's mass through an arc. On the FF-1 "
      f"the pod travels {swing_travel:.0f} mm "
      f"aft between stowed and deployed, and that moves the centre of gravity "
      f"**{ae['cg_shift_swing_mm']:+.1f} mm — {ae['cg_shift_swing_mac']:+.1f} % "
      f"of mean chord**. Static margin falls from 20 % to "
      f"{20 - ae['cg_shift_swing_mac']:.0f} % the moment the turbine deploys, "
      f"which is at the aft CG limit.\n")
    A("\nA mast that only moves vertically shifts the CG by **nothing**. Two "
      "other things come free with it:\n")
    A("\n- **The rotor axis stays aligned with the flow through the whole "
      "travel.** A swing arm at mid-stroke has the rotor at 45° to the "
      "airstream, which is a large asymmetric blade load exactly when the "
      "mechanism is least supported. That is what limits the speed at which "
      "you can move a swing arm.\n"
      "- **Deployment depth becomes a continuous variable.** The mast can be "
      "part-extended, so the turbine's exposure — and therefore the drag it "
      "costs — can be modulated instead of being on or off.\n")
    A("\nWhat it costs is mass and parts: "
      f"**{ae['dmass_g']:+.0f} g**, taking sink rate from "
      f"{ae['base']['sink']:.3f} to {ae['mast']['sink']:.3f} m/s "
      f"({ae['sink_pct']:+.1f} %). Endurance still closes above eight hours.\n")

    A("\n## 2. The stroke is set by the rotor, and nothing else\n")
    A(_t([["Rotor radius", f"{g['rotor_R']:.0f} mm"],
          ["Disc clearance from the body",
           f"{g['clearance']:.0f} mm ({MAST['clearance_frac']:.2f} × rotor Ø)"],
          ["Stowed package radius",
           f"{MAST['stowed_env_r']:.0f} mm (nacelle + blades folded aft over it)"],
          ["**Required stroke**", f"**{g['stroke']:.0f} mm**"]], ["", ""]))
    A("\nThat sum is the whole sizing problem, and it does not care about stage "
      "count, station or fuselage depth — move any of those and the stroke "
      "stays at "
      f"{g['stroke']:.0f} mm. The only lever is the rotor, and shrinking it is "
      "not available: turbine power goes as R², so dropping to a 110 mm rotor "
      "costs 46 % of the output and takes Mode C (emergency power) below the "
      "hotel load, which is the mode that justifies carrying the turbine at "
      "all.\n")
    A("\nClearance is the one soft number. 0.20 × diameter keeps the disc out "
      "of the fuselage boundary layer (about 12 mm at this station) and out of "
      "the worst of the flow curvature under the belly. Tightening it to 0.12 "
      "would save 12 mm of stroke at the cost of a non-uniform inflow across "
      "the disc — 1-per-rev blade loading, lost C_p, and noise.\n")

    A("\n## 3. Why it has to telescope\n")
    A(f"The fuselage is **{f['h']:.0f} mm deep** at station x = "
      f"{MAST['station_x']:.0f} mm (deck {f['deck']:.1f}, keel {f['keel']:.1f}). "
      f"A single sliding mast needs its own length plus the stroke: retracted, "
      f"its top ends up about {g['stroke']:.0f} mm above where its deployed top "
      f"was, so it would stand roughly 125 mm proud of the deck. There is "
      f"nowhere to put that.\n")
    A("\n**Telescoping is not a refinement here — it is the only way "
      f"{g['stroke']:.0f} mm of travel fits inside {f['h']:.0f} mm of "
      "airframe.** Stage count then trades three ways: more stages nest "
      "shorter and need less actuator stroke, but multiply the input force and "
      "add joint lash.\n\n")
    A(_t([[r["n"], f"{r['stage_travel']:.1f}", f"{r['nested']:.1f}",
           f"{r['fairing_depth']:.1f}",
           "fits" if r["fits"] else f"**{r['nested'] - r['avail']:.0f} mm too tall**"]
          for r in min_stages(limit=5)],
         ["Stages", "Travel per stage (mm)", "Nested height (mm)",
          "Ventral fairing (mm)", f"vs {f['h']:.0f} mm available"]))
    A(f"\n**Three stages.** Two will not nest; four fits but drives the "
      f"actuator force past what a micro actuator delivers.\n")

    A("\n## 4. Mast\n")
    secs = tube_sizes()
    A(_t([["Station", f"x = {MAST['station_x']:.0f} mm, on the wing "
           "trailing-edge bulkhead"],
          ["Sections (fore-aft × lateral)",
           " → ".join(f"{a:.1f}×{b:.1f}" for a, b in secs) + " mm"],
          ["Wall", f"{MAST['wall']:.1f} mm carbon"],
          ["Stage length", f"{g['stage_len']:.1f} mm"],
          ["Retained overlap", f"{dr['overlap_mm']:.1f} mm at full extension"],
          ["Nested height", f"{g['nested']:.1f} mm (fits under the deck)"],
          ["Nacelle travel", f"z {g['z_stow']:.1f} → {g['z_deployed']:.1f} mm"],
          ["Deployed rotor disc",
           f"z {g['disc_top']:.0f} to {g['disc_bot']:.0f} mm "
           f"({g['below_keel']:.0f} mm below the keel)"],
          ["Ventral fairing", f"{g['fairing_depth']:.1f} mm deep"]], ["", ""]))
    A("\n**The sections are rectangular, deep fore-aft.** Three reasons, and "
      "the first is the one that decides it:\n\n"
      "1. The air load is fore-aft, so that is the axis that needs the second "
      "moment. Round tubes of the same mass put the mast's first bending mode "
      "on top of the rotor's 1-per-rev — see §6.\n"
      "2. A non-circular section reacts the generator's torque without a "
      "keyway or a splined joint.\n"
      "3. The flat faces are where the racks go.\n")

    A("\n## 5. Drive\n")
    A("```\n"
      "micro linear actuator  (horizontal, along the fuselage)\n"
      "        |\n"
      "        |  1:1 bellcrank, 90 deg offset, equal arms\n"
      "        v\n"
      "stage 1 head  (Scotch-yoke slot -> pure vertical translation)\n"
      "        |\n"
      "        |  rack - pinion - rack doubler\n"
      "        v\n"
      "stage 2   (2 x stage 1)\n"
      "        |\n"
      "        |  rack - pinion - rack doubler\n"
      "        v\n"
      "stage 3   (3 x stage 1)  ->  nacelle, generator, rotor\n"
      "```\n")
    A("\nA pinion carried on stage 1 meshes with a rack fixed to the trunk and "
      "a rack on stage 2, so stage 2 moves twice stage 1. A second pinion on "
      "stage 2 meshes with racks on stages 1 and 3, giving 3×. Module 0.4, "
      "Ø8 mm pinions.\n")
    A("\nRack and pinion rather than cable reeving on purpose: it drives "
      "**positively in both directions** with one element per joint, there is "
      "nothing to tension or re-tension, and there are no dead centres. A "
      "reeved telescope needs a separate extend and retract cable per stage — "
      "four cables and four sheaves here — and every one of them is a "
      "creep-and-stretch item.\n")
    A("\nAn equal-armed bellcrank with its arms 90° apart gives exactly 1:1 "
      "between the actuator's horizontal travel and the yoke's vertical "
      "travel, for any sweep angle. Arms "
      f"{C.RAT_MAST['actuator']['arm_mm']:.0f} mm, pivot at x = "
      f"{C.RAT_MAST['actuator']['pivot'][0]:.0f} mm.\n")

    A("\n### 5.1 Loads\n")
    d = dr["drag"]
    A(_t([["Air load at " + f"{MAST['v_retract']:.0f} m/s, rotor braked",
           f"{d['total']:.2f} N",
           f"rotor {d['rotor']:.2f} + mast {d['mast']:.2f} + nacelle {d['nacelle']:.2f}"],
          ["Worst bushing couple",
           f"{max(j['bush_N'] for j in dr['joints']):.0f} N",
           "moment / overlap at the lowest joint"],
          ["Joint friction, all stages", f"{dr['friction_N']:.2f} N",
           f"μ = {MAST['mu_bush']:.2f}, PTFE-lined on hard-anodised"],
          ["Moving mass", f"{dr['moving_mass_g']:.0f} g",
           "rotor, generator, nacelle, stages 1–3"],
          ["Weight at " + f"{MAST['n_manoeuvre']:.0f} g",
           f"{dr['weight_N']:.2f} N", "retract during a manoeuvre"],
          ["**Design load at the mast**", f"**{dr['design_load_N']:.1f} N**",
           f"×{MAST['stiction_factor']:.1f} for stiction and contamination"],
          ["**At the actuator**", f"**{dr['actuator_force_N']:.1f} N**",
           f"×3 stages / {MAST['eta_chain'] * MAST['eta_bellcrank']:.2f} chain efficiency"]],
         ["", "", "basis"]))

    A("\n### 5.2 Actuator selection\n")
    A("Class figures for common micro linear actuators. **Verify against the "
      "current datasheet before ordering** — these are the numbers to size "
      "against, not a price list.\n\n")
    A(_t([[a["name"], f"{a['stroke']:.0f}", f"{a['force']:.0f}", f"{a['mass']:.0f}",
           f"{a['body']:.0f}", f"{a['deploy_s']:.1f}", f"{a['margin']:.1f}×",
           "**use this**" if a["ok"] else
           ("stroke short" if not a["stroke_ok"] else
            ("thin margin" if a["margin"] < 2 else "too heavy"))]
          for a in pick_actuator(dr)],
         ["Actuator class", "Stroke (mm)", "Force (N)", "Mass (g)",
          "Body (mm)", "Deploy (s)", "Margin", ""]))
    A(f"\n**L12-50 at 210:1.** {dr['actuator_force_N']:.0f} N needed against "
      f"80 N available — and the design load already carries a "
      f"{MAST['stiction_factor']:.1f}× stiction factor, so the margin on the "
      f"physical load is nearer 3×. Deployment takes about "
      f"{g['stage_travel'] / 6.0:.0f} s, which is fine: nothing about "
      "deploying this turbine is time-critical, including Mode C.\n")
    A("\nIf the build comes in heavy or the bushings measure worse than "
      "μ = 0.12, the fix is a **1.4:1 bellcrank with an L12-100**: the "
      "actuator then moves 60 mm for the same 42.7 mm at the yoke and the "
      "force requirement drops to about 29 N — 2.7× margin, for 16 g.\n")

    A("\n> **The actuator's lead screw is the only thing holding the mast in "
      "flight.** It must be a self-locking ratio (100:1 or 210:1, not 50:1), "
      "and that has to be verified on the bench with power removed and the "
      "full air load applied to the extended mast — not taken from the "
      "datasheet.\n")

    A("\n## 6. Structure and resonance\n")
    A(_t([["First bending mode", f"{st['f1_hz']:.0f} Hz"],
          ["Rotor 1-per-rev at the top of the band",
           f"{st['rotor_1p_hz']:.0f} Hz ({st['rotor_rpm']:.0f} rpm at 25 m/s)"],
          ["**Frequency ratio**", f"**{st['freq_ratio']:.1f}×** (want > 1.5)"],
          ["Tip deflection, elastic", f"{st['defl_mm']:.2f} mm under the air load"],
          ["Tip deflection, joint lash",
           f"{st['lash_mm']:.2f} mm at 0.04 mm per side"]], ["", ""]))
    A("\nThis is the trap in the whole design. An unbalanced rotor is a "
      "rotating force at 1P, and a cantilevered mast carrying it is a tuning "
      "fork. Round tubes of comparable mass put the first mode within a few "
      "hertz of 1P — dead on resonance across the normal operating band. The "
      "rectangular sections move it clear by a factor of "
      f"{st['freq_ratio']:.1f}.\n")
    A("\nBack it up in the controller anyway. The generator's field-oriented "
      "controller sets the electrical load and therefore the rotor speed, so "
      "an **RPM avoid-band costs nothing** — exclude ±8 % around any measured "
      "resonance and let the controller step over it. Measure the real "
      "frequency with a ping test on the assembled, extended mast before the "
      "first spin-up; do not trust the calculation.\n")

    A("\n## 7. Control, interlocks and failure\n")
    A("The turbine control law in [DESIGN.md](DESIGN.md) §6.1 is unchanged. "
      "The mast adds these:\n\n")
    A(_t([["Do not move the mast above " + f"{MAST['v_retract']:.0f} m/s",
           "Air load and bushing friction both scale with V²; above this the "
           "actuator margin goes"],
          ["Brake the rotor electrically before retracting",
           "Short the generator phases through the controller. A spinning "
           "rotor entering the bay is a blade strike, and it is gyroscopic"],
          ["Full retract below 25 m AGL, as a hardware failsafe",
           f"The rotor hangs {g['below_keel']:.0f} mm below the keel and there "
           "is no landing gear. Wire it independent of the scripting engine"],
          ["Two end sensors, not one",
           "The actuator's own potentiometer plus a Hall sensor at each end of "
           "the mast travel. An interlock on a single sensor is not an interlock"],
          ["Magnetic detent at stowed",
           f"{C.RAT_MAST['detent']} — stops the stack rattling and creeping "
           "without adding a breakout mechanism"]],
         ["Rule", "Why"]))
    A("\n**If the actuator fails with the mast extended, the aircraft still "
      "has to land.** Design the trunk-to-bulkhead joint as the weak link: a "
      "defined shear load that lets the mast break away cleanly on a belly "
      "landing, with the mast on a lanyard so it does not become debris. That "
      "confines the damage to two cheap parts. A spring-loaded fail-safe "
      "retract was considered and rejected — a spring strong enough to "
      "retract against friction adds its full force to every deployment, and "
      "the actuator margin will not carry it.\n")

    A("\n## 8. Build and rigging notes\n")
    A("1. Bond the trunk between two bulkheads, not into the shell. You are "
      "cutting a "
      f"{MAST['trunk_x']:.0f} × {MAST['trunk_y']:.0f} mm hole through the keel "
      "at the station carrying the wing's rear load path — it needs a frame, "
      "and a carbon rim around the opening.\n"
      "2. Assemble and cycle the mast **dry, out of the aircraft, 200 times** "
      "before it goes in. Anything that will bind, binds early.\n"
      "3. Set the rack mesh with the stages fully extended, then check backlash "
      "retracted. It is the extended end that carries load.\n"
      "4. Rig the bellcrank so the yoke bottoms on its hard stop 0.5 mm before "
      "the actuator reaches its own end of travel — the stop takes the load, "
      "not the screw.\n"
      "5. Fit the wiper seal at the trunk mouth last, and check it does not "
      "add measurable breakout force. An open trunk is a ram-air path straight "
      "into the fuselage.\n"
      "6. Ping-test the extended mast and record the frequency. Compare with "
      f"the predicted {st['f1_hz']:.0f} Hz before the first spin-up.\n")
    return "\n".join(o)


def main():
    path = os.path.join(ROOT, "docs", "VERTICAL_MAST.md")
    with open(path, "w") as fh:
        fh.write(report())
    print(f"wrote {os.path.relpath(path, ROOT)} "
          f"({os.path.getsize(path) / 1024:.1f} kB)")


if __name__ == "__main__":
    main()
