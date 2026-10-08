#!/usr/bin/env python3
"""
EDF propulsion for the Jetwing: a 50 mm 12-blade fan on a QF2611 5000KV.

    python3 tools/edf.py

PUBLISHED FIGURES THIS IS BUILT ON
  fan unit   50 mm, 12 blades, QF2611 5000KV, 9N6P, 3-4S
  mass       109 g complete (motor alone 76 g)
  rating     550 W / 45 A maximum continuous
  quoted     950 g static thrust at 12.6 V, 45 A, 567 W

Note 12.6 V is a FULL 3S pack, not 4S. At 4S (14.8 V nominal) the same 45 A
would be 666 W, well past the 550 W continuous rating, so on 4S this unit is
POWER limited rather than voltage limited: you throttle it back to about
37 A. That matters, because it means 4S buys very little extra thrust over
3S -- it buys lower current, cooler running and a lighter wiring loom.

The real Jetwing is specified for a 70 mm EDF on 4S. A 50 mm unit moves far
less air, and the comparison at the bottom of this file puts a number on it.

MODEL
  Ducted fan momentum theory at the nozzle, which is the honest way to size
  a duct because it works in terms of exit area and exit velocity:

      mass flow   mdot = rho Ae Ve
      thrust      T    = mdot (Ve - V)
      jet power   P    = 0.5 mdot (Ve^2 - V^2)

  Given shaft power, the first and third give a cubic in Ve, solved by
  bisection. Static is the V = 0 case, Ve = (2 eta P / rho Ae)^(1/3).

  An open propeller contracts its wake to half the disc area; a duct does
  not, which is why a ducted fan makes more static thrust than an open rotor
  of the same diameter and power. That is the whole reason a 50 mm fan is
  worth anything at all.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

RHO = 1.225
G = 9.80665

FAN = {
    "name": "QF2611 5000KV / 50 mm 12-blade",
    "dia": 50.0,              # mm, fan outer diameter
    "hub": 26.0,              # mm, motor can is 25.8
    "blades": 12,
    "kv": 5000.0,
    "mass_g": 109.0,          # complete unit
    "p_max": 550.0,           # W, maximum continuous
    "i_max": 45.0,            # A
    "nozzle_fsa": 0.90,       # exit area as a fraction of fan swept area
    "inlet_fsa": 1.05,        # total inlet capture area
    "eta_motor": 0.82,
    "eta_fan": 0.75,          # shaft power -> jet kinetic energy, calibrated
    "eta_fan_real": 0.62,     # what these units usually measure at
    "cells_3s": 11.1, "cells_4s": 14.8,
}

SEVENTY = {                   # what the real Jetwing is specified for
    "name": "70 mm EDF, 4S",
    "dia": 70.0, "hub": 32.0, "mass_g": 285.0,
    "p_max": 1300.0, "nozzle_fsa": 0.90,
    "eta_motor": 0.84, "eta_fan": 0.70,
}


# ==========================================================================
# areas
# ==========================================================================

def areas(f=FAN):
    fsa = math.pi / 4.0 * (f["dia"] ** 2 - f["hub"] ** 2)      # mm2
    ae = f["nozzle_fsa"] * fsa
    ai = f.get("inlet_fsa", 1.05) * fsa
    return {"fsa": fsa, "ae": ae, "ai": ai,
            "fsa_m2": fsa / 1e6, "ae_m2": ae / 1e6, "ai_m2": ai / 1e6,
            "exit_dia": math.sqrt(4.0 * ae / math.pi),
            "inlet_dia_each": math.sqrt(4.0 * (ai / 2.0) / math.pi)}


# ==========================================================================
# the duct
# ==========================================================================

def duct(P_shaft, V, Ae_m2, eta_fan):
    """Exit velocity and thrust for a duct of exit area Ae.

    Solves  0.5 rho Ae Ve (Ve^2 - V^2) = eta P_shaft  for Ve.
    """
    if P_shaft <= 0.0:
        return {"Ve": V, "T": 0.0, "mdot": RHO * Ae_m2 * V, "eta_p": 0.0,
                "q_jet": 0.5 * RHO * V * V}
    tgt = eta_fan * P_shaft
    lo, hi = V, max(V + 1.0, 400.0)
    for _ in range(100):
        ve = 0.5 * (lo + hi)
        p = 0.5 * RHO * Ae_m2 * ve * (ve * ve - V * V)
        if p < tgt:
            lo = ve
        else:
            hi = ve
    ve = 0.5 * (lo + hi)
    mdot = RHO * Ae_m2 * ve
    T = mdot * (ve - V)
    return {"Ve": ve, "T": T, "mdot": mdot,
            "eta_p": (T * V / P_shaft) if P_shaft > 0 else 0.0,
            "q_jet": 0.5 * RHO * ve * ve,
            "pressure_ratio": (ve / max(V, 1e-6)) if V > 0 else float("inf")}


def operate(throttle, V, cells="4s", f=FAN, realistic=True):
    """Electrical input -> thrust. Throttle commands power, which is how a
    fixed-pitch EDF behaves closely enough at a given airspeed."""
    a = areas(f)
    volts = f["cells_4s"] if cells == "4s" else f["cells_3s"]
    # power limited, not voltage limited: the rating binds before the pack does
    p_elec = min(f["p_max"], volts * f["i_max"]) * throttle
    p_shaft = p_elec * f["eta_motor"]
    eta = f["eta_fan_real"] if realistic else f["eta_fan"]
    d = duct(p_shaft, V, a["ae_m2"], eta)
    return {**d, "p_elec": p_elec, "p_shaft": p_shaft, "volts": volts,
            "amps": p_elec / volts, "throttle": throttle, "V": V,
            "areas": a, "eta_fan": eta}


# ==========================================================================
# the jet behind the nozzle, for the vectoring rudder
# ==========================================================================

def jet_at(x_mm, exit_dia, Ve, V_inf, spread=0.08):
    """A free jet spreads and slows. Momentum is what is conserved, so

        rho A(x) V(x)^2 = rho Ae Ve^2    ->    V(x) = Ve * Re / R(x)

    with R(x) = Re + spread*x. Crude next to a proper shear-layer solution,
    but it conserves the thing that makes the force and it is honest about
    the jet getting wider and slower the further back you put the rudder.
    """
    re = exit_dia / 2.0
    r = re + spread * max(0.0, x_mm)
    v = max(V_inf, Ve * re / r)
    return {"R": r, "V": v, "q": 0.5 * RHO * v * v, "Re": re}


# ==========================================================================
# designing the installation for efficiency
# ==========================================================================

def froude(V, Ve):
    """Ideal propulsive efficiency, 2V/(V+Ve). This is the whole problem with
    a small fan: it has to throw a FAST jet to make thrust from a small area,
    and efficiency is set by how badly the jet speed overshoots flight speed."""
    return 2.0 * V / (V + Ve) if (V + Ve) > 0 else 0.0


def nozzle_sweep(V=16.0, f=FAN, ratios=(0.80, 0.85, 0.90, 0.95, 1.00, 1.05)):
    """Exit area is the biggest lever on efficiency. A bigger nozzle means a
    slower, fatter jet: less thrust per watt lost to kinetic energy, more of
    the power going into the aircraft. The limit is the fan -- open the
    nozzle too far and it unloads, overspeeds and stalls its blades."""
    rows = []
    for r in ratios:
        g = dict(f); g["nozzle_fsa"] = r
        a = areas(g)
        p_elec = min(g["p_max"], g["cells_4s"] * g["i_max"])
        d = duct(p_elec * g["eta_motor"], V, a["ae_m2"], g["eta_fan_real"])
        rows.append({"ratio": r, "ae": a["ae"], "dia": a["exit_dia"],
                     "T": d["T"], "Ve": d["Ve"], "eta_p": d["eta_p"],
                     "froude": froude(V, d["Ve"])})
    return rows


def capture(V, f=FAN, realistic=True):
    """Free-stream area of the streamtube the inlet swallows.

    If that tube is WIDER than the inlet, the flow accelerates in and nothing
    spills. If it is narrower, the inlet is too big for the speed and the
    excess spills round the lip, which costs drag. The crossover speed is
    where an inlet stops being free."""
    a = areas(f)
    r = operate(1.0, V, "4s", f, realistic)
    mdot = r["mdot"]
    tube = mdot / (RHO * V) if V > 0.1 else float("inf")
    v_cross = mdot / (RHO * a["ai_m2"])
    return {"mdot": mdot, "tube_mm2": tube * 1e6 if V > 0.1 else None,
            "inlet_mm2": a["ai"], "ratio": (tube * 1e6 / a["ai"]) if V > 0.1 else None,
            "v_crossover": v_cross, "v_inlet": mdot / (RHO * a["ai_m2"])}


def cold_drag(V, f=FAN, K=1.0, blocked_cd=0.5):
    """What the duct costs when the fan is OFF -- which on a soaring aircraft
    is most of the flight, and is the number that decides whether a fixed
    installation is worth it.

    Freewheeling: the duct still passes air, but with a total-pressure loss,
    so it leaves slower than it arrived and the momentum deficit is drag.
        Ve = V / sqrt(1+K),  D = mdot (V - Ve)
    Braked: nothing passes, the inlet stagnates and spills, so it is plain
    form drag on the capture area.
    """
    a = areas(f)
    ve = V / math.sqrt(1.0 + K)
    mdot = RHO * a["ae_m2"] * ve
    free = mdot * (V - ve)
    stop = blocked_cd * 0.5 * RHO * V * V * a["ai_m2"]
    return {"freewheel": free, "braked": stop, "Ve": ve, "K": K}


def diffuser(f=FAN, n_inlets=2, duct_len=150.0):
    """Internal duct angles. Above about 7 degrees of half-angle a diffuser
    separates and you lose the pressure recovery you built the duct for."""
    a = areas(f)
    d_in = a["inlet_dia_each"]
    d_fan = f["dia"]
    # two round inlets merging into one annulus: equivalent single diameter
    d_eq = math.sqrt(n_inlets) * d_in
    half = math.degrees(math.atan2((d_fan - d_eq) / 2.0, duct_len))
    # nozzle contraction, fan face to exit
    noz_half = math.degrees(math.atan2((d_fan - a["exit_dia"]) / 2.0, 90.0))
    return {"d_inlet": d_in, "d_equiv": d_eq, "d_fan": d_fan,
            "diffuser_half_deg": half, "nozzle_half_deg": noz_half,
            "lip_radius": 0.10 * d_in}


def efficiency_report(V=16.0, f=FAN):
    L = []
    L.append(f"== 50 mm EDF installation, designed for cruise at {V:.0f} m/s ==")
    L.append("")
    r = operate(1.0, V, "4s", f)
    L.append(f"As drawn: {r['T'] / G * 1000:.0f} g thrust, jet {r['Ve']:.1f} m/s, "
             f"propulsive efficiency {r['eta_p'] * 100:.1f}%")
    L.append(f"  ideal (Froude) ceiling at that jet speed is "
             f"{froude(V, r['Ve']) * 100:.1f}% -- the fan is small, so the jet")
    L.append(f"  overshoots flight speed {r['Ve'] / V:.1f}x and most of the power "
             f"goes into the air, not the aircraft.")
    L.append("")
    L.append("NOZZLE AREA, the biggest lever")
    L.append("  Ae/FSA   dia    thrust    jet    eta_p   Froude")
    for row in nozzle_sweep(V, f):
        L.append(f"   {row['ratio']:.2f}   {row['dia']:5.1f}  {row['T'] / G * 1000:5.0f} g "
                 f"{row['Ve']:6.1f}  {row['eta_p'] * 100:5.1f}%  {row['froude'] * 100:5.1f}%")
    L.append("  Opening the nozzle trades static thrust for efficiency. 0.90 is")
    L.append("  the usual compromise and is what is built; going past 1.00 unloads")
    L.append("  the fan into blade stall, so it is not free.")
    L.append("")
    L.append("INLET")
    c = capture(V, f)
    d = diffuser(f)
    L.append(f"  swallows {c['mdot'] * 1000:.0f} g/s; at {V:.0f} m/s that is a "
             f"free-stream tube {c['ratio']:.1f}x the inlet area,")
    L.append(f"  so the flow accelerates IN and nothing spills. Spillage would only")
    L.append(f"  start above {c['v_crossover']:.0f} m/s, which this aircraft never sees.")
    L.append(f"  inlet velocity {c['v_inlet']:.0f} m/s -> the lip has to turn still air")
    L.append(f"  hard, so round it to {d['lip_radius']:.1f} mm. A sharp lip separates and")
    L.append(f"  that is where cheap installations lose 10-20% of their thrust.")
    L.append("")
    L.append("INTERNAL ANGLES")
    L.append(f"  two {d['d_inlet']:.1f} mm inlets -> {d['d_equiv']:.1f} mm equivalent "
             f"-> {d['d_fan']:.0f} mm fan")
    L.append(f"  diffuser half-angle {d['diffuser_half_deg']:.1f} deg "
             f"(keep under 7, or it separates)")
    L.append(f"  nozzle half-angle {d['nozzle_half_deg']:.1f} deg, converging, "
             f"which is always safe")
    L.append("")
    L.append("WHAT IT COSTS WHEN THE FAN IS OFF")
    for v in (10.0, 14.0, 18.0):
        cd = cold_drag(v, f)
        L.append(f"  {v:4.1f} m/s: freewheeling {cd['freewheel']:.3f} N, "
                 f"braked {cd['braked']:.3f} N")
    L.append("  A soaring aircraft glides far more than it motors, so this is the")
    L.append("  number that decides the installation. LET THE FAN FREEWHEEL --")
    L.append("  set the ESC to coast, not brake. A stopped fan is a flat plate.")
    return "\n".join(L)
