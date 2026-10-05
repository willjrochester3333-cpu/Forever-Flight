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
