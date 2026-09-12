#!/usr/bin/env python3
"""
Generate a synthetic FF-1 flight log, for testing the analyser and for having
something to run before the aircraft exists.

The flight is flown against the REAL design model in tools/analysis.py, then
sensor noise is added. So the analyser's fitted drag polar can be checked
against a known truth: it should recover C_D0 and Oswald e to within a few
percent. If it does not, the fitter is wrong.

The profile follows the phase-1 flight card in docs/DESIGN.md section 11:
a motor climb, then a series of stabilised glides at 7.5 / 8.5 / 9.5 / 11 /
13 m/s, then thermal soaring with three turbine-deployed regenerative
descents.

Run:  python3 tools/simulate_flight.py [out.csv]
"""

from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analysis as AN
import config as C

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RATE_HZ = 2.0
SEED = 20260912

POLAR_LEGS = [(7.5, 55), (8.5, 55), (9.5, 60), (11.0, 55), (13.0, 50)]


def isa(h):
    """ISA temperature (K) and density at altitude h (m)."""
    T = 288.15 - 0.0065 * h
    rho = 1.225 * (T / 288.15) ** 4.2561
    return T, rho


def true_sink(V, W, S, rho, rat=False):
    """Sink rate from the design polar at the local density."""
    CL = 2.0 * W / (rho * S * V * V)
    if CL > C.DRAG["cl_max"] * 1.05:
        return None, CL
    CD = AN.polar(CL, motor=False, rat=rat)
    return 0.5 * rho * S * V ** 3 * CD / W, CL


def main(out_path=None):
    rnd = random.Random(SEED)
    m = AN.mass_rollup()
    mass = m["mtow_g"] / 1000.0
    W = mass * AN.G
    S = AN.geom()["S"]
    hotel = sum(h[1] for h in C.HOTEL)
    e_chain = C.EFF["prop"] * C.EFF["motor"] * C.EFF["esc"]
    bat = C.BATTERY
    cap_wh = bat["cells_series"] * bat["cells_parallel"] * bat["cell_wh"]

    site = C.MISSION["sites"][0]
    lat, doy, clearness = site[1], site[2], site[3]
    launch_hour = 10.0

    dt = 1.0 / RATE_HZ
    rows = []
    t = 0.0
    alt = 2.0
    used_wh = 0.0
    v_meas = 0.0

    # ---- flight plan ---------------------------------------------------
    # Fixed-duration phases up front, then an ALTITUDE-driven soaring state
    # machine so the aircraft stays inside its soar band instead of climbing
    # away forever.
    band = C.MISSION["soar_band_m"]
    ceiling, floor = band[1] - 20.0, band[0] + 70.0

    phases = [("ground", 20, None, "ground"),
              ("launch", 25, 11.0, "climb"),
              ("climb to 450", None, 11.0, "climb")]      # None = until alt target
    for v, d in POLAR_LEGS:
        phases.append(("settle", 14, v, "settle"))
        phases.append((f"polar {v:.1f}", d, v, "polar"))
    phases.append(("reclimb", None, 11.0, "climb"))

    alt_targets = {2: 450.0, len(phases) - 1: 520.0}
    total_budget_s = 2.6 * 3600

    turb_smooth = 0.0
    cycle = 0

    def step(mode, v_target, label):
        """One time step. Returns the sampled row; mutates alt/used_wh/t."""
        nonlocal alt, used_wh, t, turb_smooth
        T_k, rho = isa(alt)
        hour = launch_hour + t / 3600.0

        irr, _el = AN.clear_sky(lat, doy, hour, clearness)
        p_solar, _ = AN.array_power(irr)
        p_solar *= 1.0 + 0.04 * rnd.gauss(0, 1)
        if rnd.random() < 0.0006:
            p_solar *= 0.45
        p_solar = max(0.0, p_solar)

        rat_out = mode == "regen"
        throttle = p_motor = p_turb = 0.0
        roll = rnd.gauss(0, 1.2)

        if mode == "ground":
            V, climb = 0.0, 0.0
            alt = 2.0
        elif mode == "climb":
            V, throttle = v_target, 0.85
            sink, _ = true_sink(V, W, S, rho, rat=False)
            climb = 2.5
            p_motor = W * (climb + (sink or 0.6)) / e_chain
        elif mode in ("polar", "settle"):
            V = v_target
            sink, _ = true_sink(V, W, S, rho, rat=False)
            climb = -(sink or 0.6)
            roll = rnd.gauss(0, 0.8)
        elif mode == "thermal":
            V = v_target
            roll = 38.0 + rnd.gauss(0, 3.0)
            w_core = 3.1 + 0.5 * math.sin(t / 47.0)
            climb = w_core - AN.circling_sink(W) + rnd.gauss(0, 0.25)
        elif mode == "regen":
            V = v_target
            sink, _ = true_sink(V, W, S, rho, rat=True)
            rp = AN.rat_power(V, rho=rho)
            climb = -((sink or 1.0) + rp["p_shaft"] / W)
            p_turb = rp["p_elec"] * (1.0 + 0.05 * rnd.gauss(0, 1))
        else:                                     # glide / search
            V = v_target
            sink, _ = true_sink(V, W, S, rho, rat=False)
            climb = -(sink or 0.6) + 0.35 * math.sin(t / 31.0)
            roll = rnd.gauss(0, 6.0)

        alt = max(2.0, alt + climb * dt)

        p_out = hotel + p_motor
        p_in = p_solar + p_turb
        used_wh += max(0.0, p_out - p_in) * dt / 3600.0
        soc = max(0.03, min(1.0, 1.0 - used_wh / (cap_wh * bat["usable_dod"])))
        volt = (3.35 + 0.85 * soc ** 0.55) * bat["cells_series"]
        curr = (p_out - p_in) / max(volt, 1.0)
        volt -= 0.06 * max(curr, 0.0)
        turb_smooth += 0.25 * (p_turb - turb_smooth)

        row = {
            "TimeUS": int(t * 1e6),
            "Alt": alt + rnd.gauss(0, 0.35),
            "Airspeed": (V + rnd.gauss(0, 0.22) + 0.15 * math.sin(t / 3.7)
                         if V > 0 else 0.0),
            "GroundSpeed": (V + rnd.gauss(0, 0.5) + 2.2 if V > 0 else 0.0),
            "Roll": roll + rnd.gauss(0, 0.7),
            "Pitch": rnd.gauss(2.0, 1.5),
            "ThrOut": max(0.0, min(1.0, throttle + rnd.gauss(0, 0.01))),
            "Volt": volt + rnd.gauss(0, 0.02),
            "Curr": curr + rnd.gauss(0, 0.05),
            "SolarW": p_solar,
            "TurbineW": max(0.0, turb_smooth),
            "MastMM": 128.0 if rat_out else 0.0,
            "Temp": T_k - 273.15 + rnd.gauss(0, 0.3),
        }
        t += dt
        return row

    # fixed phases
    for idx, (label, dur, v_target, mode) in enumerate(phases):
        if dur is None:
            target = alt_targets[idx]
            guard = 0
            while alt < target and guard < 20000:
                rows.append(step(mode, v_target, label))
                guard += 1
        else:
            for _i in range(int(dur / dt)):
                rows.append(step(mode, v_target, label))

    # altitude-driven soaring block
    while t < total_budget_s:
        while alt < ceiling and t < total_budget_s:
            rows.append(step("thermal", 9.0, "thermal"))
        if cycle % 3 == 2:                       # every third descent is a regen run
            while alt > floor and t < total_budget_s:
                rows.append(step("regen", 14.0, "regen"))
        else:
            while alt > floor and t < total_budget_s:
                rows.append(step("glide", 10.5, "search"))
        cycle += 1

    for _i in range(int(150 / dt)):              # approach
        rows.append(step("glide", 9.5, "approach"))

    out_path = out_path or os.path.join(ROOT, "data", "example_flight.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cols = list(rows[0].keys())
    with open(out_path, "w") as fh:
        fh.write(",".join(cols) + "\n")
        for r in rows:
            fh.write(",".join(
                str(r[c]) if c == "TimeUS" else f"{r[c]:.3f}" for c in cols) + "\n")

    print(f"wrote {os.path.relpath(out_path, ROOT)}")
    print(f"  {len(rows):,} rows at {RATE_HZ:.0f} Hz, "
          f"{t / 60:.0f} min, {os.path.getsize(out_path) / 1e6:.2f} MB")
    print(f"  flown against the design polar: C_D0 = {AN.cd0_clean():.4f}, "
          f"e = {C.DRAG['oswald_e']:.2f}, mass = {mass:.3f} kg")
    print(f"  polar legs at {', '.join(f'{v:.1f}' for v, _d in POLAR_LEGS)} m/s")
    return out_path


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
