#!/usr/bin/env python3
"""
Export a compact, riggable version of the FF-1 model for the web viewer.

The viewer animates deployment, so the pods leave here in their own local
frames (pivot at the origin, arm along +X) and the blades leave as raw radial
meshes. The viewer composes the transforms, which means the deployment travel
in the browser is the same travel the geometry model uses -- there is no second
source of truth for where anything is.

Vertices are quantised to int16 over the model bounding box (~0.03 mm), then
base64'd. Output: viewer/model.json
"""

from __future__ import annotations

import base64
import json
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analysis as AN
import build_model as B
import config as C
import mesh as M

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Coarser than the STL: the viewer wants bytes, not micrometres.
B.LOD.update({"airfoil_n": 40, "wing_bays": 3, "fuse_pts": 20,
              "blister_pts": 14, "nacelle_pts": 12, "blade_steps": 6,
              "vtail_steps": 5, "cell_nx": 1, "cell_nc": 3,
              "strut_steps": 4, "turret_steps": 10})


def local_frame(pivot, angle_deg):
    """World -> pod-local: pivot at the origin, arm along +X."""
    return M.compose(M.translate(-pivot[0], -pivot[1], -pivot[2]),
                     M.rot_y(angle_deg))


def pack(meshes):
    """Quantise a list of (name, colour, Mesh) into base64 buffers."""
    lo = [1e30] * 3
    hi = [-1e30] * 3
    for (_n, _c, m) in meshes:
        for p in m.v:
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    span = max(hi[i] - lo[i] for i in range(3)) or 1.0
    scale = span / 65534.0

    out = []
    for (name, colour, m) in meshes:
        if not m.f:
            continue
        vb = bytearray()
        for (x, y, z) in m.v:
            for v, o in ((x, lo[0]), (y, lo[1]), (z, lo[2])):
                vb += struct.pack("<H", max(0, min(65534, int(round((v - o) / scale)))))
        wide = len(m.v) > 65535
        ib = bytearray()
        for tri in m.f:
            for i in tri:
                ib += struct.pack("<I" if wide else "<H", i)
        out.append({"name": name, "color": colour,
                    "nv": len(m.v), "nt": len(m.f), "wide": wide,
                    "v": base64.b64encode(bytes(vb)).decode(),
                    "i": base64.b64encode(bytes(ib)).decode()})
    return out, {"origin": lo, "scale": scale}


def _harvest_spec():
    """Soaring-cycle harvest: the grid, and where it converts to endurance."""
    m = AN.mass_rollup()
    W = m["mtow_g"] / 1000.0 * AN.G
    site = C.MISSION["sites"][0]
    grid = [{"w": row["w"],
             "cells": [{"f": c["f"], "h": round(c["h"], 2),
                        "soarable": c["soarable"],
                        "rotor_limited": c["rotor_limited"]}
                       for c in row["cells"]]}
            for row in AN.harvest_grid(W)]
    spill = []
    for sc, lab in [(1.00, "clear sky, clean array"),
                    (0.55, "thin overcast"),
                    (0.40, "one of three strings failed"),
                    (0.30, "heavy overcast"),
                    (0.20, "array badly degraded")]:
        a = AN.simulate(site, use_turbine=False, solar_scale=sc)
        b = AN.simulate(site, use_turbine=True, solar_scale=sc)
        dt = 30 / 3600
        sp = sum(x["p_harvest"] * dt for x in b["trace"]
                 if x["e"] >= b["usable_wh"] - 1e-6)
        spill.append({"pct": round(sc * 100), "label": lab,
                      "off": round(a["endurance_h"], 2),
                      "on": round(b["endurance_h"], 2),
                      "gain": round(b["endurance_h"] - a["endurance_h"], 2),
                      "spilled": round(100 * sp / max(b["harvest_wh"], 1e-9))})
    good = AN.soaring_harvest(W, 3.2, 0.42)
    return {"grid": grid, "spill": spill,
            "good": round(good["harvest_w"], 2),
            "v_opt": good["v"],
            "wh": round(AN.simulate(site)["harvest_wh"], 1)}


def _mast_spec():
    """VDM-1 sizing, straight from tools/mast.py."""
    import mast as MS
    g, dr, st = MS.geometry(), MS.drive(), MS.structure()
    ae = MS.aircraft_effect()
    act = [a for a in MS.pick_actuator(dr)]
    return {
        "stationX": MS.MAST["station_x"],
        "stroke": round(g["stroke"], 1),
        "actuator_stroke": round(g["stage_travel"], 1),
        "stages": g["n_stages"],
        "rotor_R": g["rotor_R"], "clearance": round(g["clearance"], 1),
        "env_r": MS.MAST["stowed_env_r"],
        "nested": round(g["nested"], 1), "avail": round(g["fuse"]["h"], 1),
        "fairing": round(g["fairing_depth"], 1),
        "z_stow": round(g["z_stow"], 1), "z_dep": round(g["z_deployed"], 1),
        "below_keel": round(g["below_keel"]),
        "sections": [[round(a, 1), round(b, 1)] for a, b in MS.tube_sizes()],
        "stage_len": round(g["stage_len"], 1),
        "overlap": round(dr["overlap_mm"], 1),
        "drag_N": round(dr["drag"]["total"], 2),
        "v_retract": MS.MAST["v_retract"],
        "friction_N": round(dr["friction_N"], 2),
        "bush_N": round(max(j["bush_N"] for j in dr["joints"])),
        "moving_g": round(dr["moving_mass_g"]),
        "weight_N": round(dr["weight_N"], 2),
        "load_N": round(dr["design_load_N"], 1),
        "need_N": round(dr["actuator_force_N"], 1),
        "f1": round(st["f1_hz"]), "onep": round(st["rotor_1p_hz"]),
        "ratio": round(st["freq_ratio"], 1),
        "lash": round(st["lash_mm"], 2), "defl": round(st["defl_mm"], 2),
        "dmass": round(ae["dmass_g"]),
        "sink_pct": round(ae["sink_pct"], 1),
        "cg_swing_mm": round(ae["cg_shift_swing_mm"], 1),
        "cg_swing_mac": round(ae["cg_shift_swing_mac"], 1),
        "actuators": [{"name": a["name"], "stroke": a["stroke"],
                       "force": a["force"], "mass": a["mass"],
                       "body": a["body"], "speed": a["speed"],
                       "margin": round(a["margin"], 1),
                       "deploy_s": round(a["deploy_s"], 1),
                       "stroke_ok": a["stroke_ok"], "ok": a["ok"]}
                      for a in act],
    }


def build():
    mo, ra = C.MOTOR, C.RAT
    wing_mesh, props = B.build_wing()

    static = [
        ("wing_stbd", "#dfe2e6", wing_mesh),
        ("wing_port", "#dfe2e6", wing_mesh.mirrored_y()),
        ("fuselage", "#c9ccd1", B.build_fuselage()),
        ("dorsal_spine", "#b6bac1", B.build_blister(C.FUSELAGE["dorsal_spine"], True, "s")),
        ("ventral_fairing", "#b6bac1", B.build_blister(C.FUSELAGE["ventral_fairing"], False, "v")),
        ("sensor_turret", "#3c4046", B.build_turret()),
        ("vtail_stbd", "#d4d7dc", B.build_vtail()),
        ("vtail_port", "#d4d7dc", B.build_vtail().mirrored_y()),
        ("solar_cells", "#121826", B.build_solar_cells()),
    ]

    # --- motor pod, in its own frame ---------------------------------------
    pyl = B.build_strut(mo["pivot"], mo["pylon_len"], mo["pylon_chord"],
                        mo["pylon_thickness"], mo["deployed_angle"], "pylon")
    pyl.apply(local_frame(mo["pivot"], mo["deployed_angle"]))

    nac = B.build_nacelle(mo["nacelle_len"], mo["nacelle_dia"],
                          mo["spinner_len"], mo["spinner_dia"], "nac")
    nac.apply(M.rot_y(-mo["thrust_angle"]))

    R = mo["prop_dia"] / 2.0
    pitch_mm = mo["prop_pitch_in"] * 25.4
    blade = B.build_blade(
        20.0, R,
        lambda r: 26.0 * (1.0 - 0.55 * (r / R - 0.45) ** 2 / 0.30)
        * (1.0 - 0.88 * max(0.0, r / R - 0.86) / 0.14),
        lambda r: math.degrees(math.atan2(pitch_mm, 2.0 * math.pi * max(r, 12.0))),
        0.10, 0.045, "prop_blade")

    # --- turbine mast (VDM-1) ----------------------------------------------
    # Everything leaves in the STOWED pose; the viewer translates each stage
    # down by (stage index x stage travel x deployment), which is exactly the
    # kinematics the geometry model uses.
    stack = B.mast_stack(0.0)
    mast_meshes = []
    for sg in stack["stages"]:
        ax, ay = sg["sec"]
        mast_meshes.append((sg["name"],
                            B.rect_tube(ax, ay, stack["x_c"],
                                        sg["z0"], sg["z1"], sg["name"])))

    # Left at the origin: the viewer positions it, exactly as it does the
    # blades, so there is one place where the hub location is decided.
    rnac = B.build_nacelle(ra["nacelle_len"], ra["nacelle_dia"],
                           ra["spinner_len"], ra["spinner_dia"], "rnac")
    rnac.apply(M.rot_y(180.0))

    RR = ra["rotor_dia"] / 2.0
    lam = ra["lambda_design"]
    tblade = B.build_blade(
        ra["hub_r"], RR,
        lambda r: ra["blade_chord"] * (1.25 - 0.55 * r / RR)
        * (1.0 - 0.85 * max(0.0, r / RR - 0.90) / 0.10),
        lambda r: max(math.degrees(math.atan2(2.0, 3.0 * lam * max(r / RR, 0.20))) - 5.0, 3.0),
        0.09, 0.035, "turbine_blade")

    mast_cols = ["#7b838e", "#4a8ba6", "#3a7a94", "#2f6a83"]
    meshes = static + [
        ("motor_pylon", "#c45c3c", pyl),
        ("motor_nacelle", "#c45c3c", nac),
        ("prop_blade", "#a8492e", blade),
        ("rat_nacelle", "#3a7a94", rnac),
        ("turbine_blade", "#2e6478", tblade),
    ] + [(n, mast_cols[min(i, 3)], m) for i, (n, m) in enumerate(mast_meshes)]
    comps, quant = pack(meshes)

    m = AN.mass_rollup()
    W = m["mtow_g"] / 1000.0 * AN.G
    g = AN.geom()
    kp = AN.key_points(W)
    pms = AN.practical_min_sink(W)
    det = AN.detection()
    sim = AN.simulate(C.MISSION["sites"][0], launch_h=9.0)
    simb = AN.simulate(C.MISSION["sites"][0], launch_h=9.0,
                       use_solar=False, allow_soar=False)
    sims = AN.simulate(C.MISSION["sites"][0], launch_h=9.0, allow_soar=False)
    simu = AN.simulate(C.MISSION["sites"][1], launch_h=9.0)
    simn = AN.simulate(C.MISSION["sites"][0], launch_h=9.0, use_turbine=False)
    L = B.solar_layout()
    st = AN.stability()
    wb = AN.wing_beam()
    p_cruise, _ = AN.elec_power_for(kp["best_glide"]["V"], W, motor=True)

    rig = {
        "motor": {
            "pivot": list(mo["pivot"]),
            "deployed": mo["deployed_angle"], "stowed": mo["stowed_angle"],
            "pylonLen": mo["pylon_len"], "nacelleOffset": mo["nacelle_offset"],
            "nacelleLen": mo["nacelle_len"], "thrustAngle": mo["thrust_angle"],
            "propR": R, "blades": mo["blades"],
            "bladeAzimuth": 78.0, "bladeFold": -78.0,
        },
        "mast": {
            "stages": [sg["name"] for sg in stack["stages"]],
            "stageTravel": stack["geom"]["stage_travel"],
            "stroke": stack["geom"]["stroke"],
            "hubStowed": list(stack["hub"]),
            "nacelleLen": ra["nacelle_len"],
            "rotorR": RR, "blades": ra["blades"],
            "bladeAzimuth": 20.0, "bladeFold": -74.0,
            "stationX": stack["x_c"],
            "actuator": C.RAT_MAST["actuator"],
        },
    }

    def rat_rows(vs):
        return [{"v": v, **{k: round(val, 2) for k, val in AN.rat_power(v).items()
                            if k in ("p_shaft", "p_elec", "extraction_drag_N", "rpm")}}
                for v in vs]

    spec = {
        "span_m": g["b"], "area_m2": round(g["S"], 4), "ar": round(g["AR"], 2),
        "mac_mm": round(g["mac"] * 1000, 1), "mtow_g": round(m["mtow_g"]),
        "wing_loading": round(W / g["S"], 1),
        "ld": round(kp["best_glide"]["LD"], 1),
        "v_bg": round(kp["best_glide"]["V"], 2),
        "sink_bg": round(kp["best_glide"]["sink"], 3),
        "sink_min": round(pms["sink"], 3), "v_min_sink": round(pms["V"], 2),
        "v_stall": round(kp["v_stall"], 2),
        "cd0": round(AN.cd0_clean(), 4),
        "hotel_w": round(sum(h[1] for h in C.HOTEL), 1),
        "cruise_w": round(p_cruise, 1),
        "array_stc": round(L["p_stc_wing_w"], 1),
        "array_area_cm2": round(L["wing_area_m2"] * 1e4),
        "array_cells": round(L["n_strips_wing"] / 3.0, 2),
        "array_chain": round(AN.array_power(1000.0)[1] * 100, 1),
        "pack_wh": round(C.BATTERY["cells_series"] * C.BATTERY["cell_wh"], 1),
        "pack_usable_wh": round(sim["usable_wh"], 1),
        "endurance_h": round(sim["endurance_h"], 1),
        "endurance_batt_h": round(simb["endurance_h"], 1),
        "endurance_solar_h": round(sims["endurance_h"], 1),
        "endurance_uk_h": round(simu["endurance_h"], 1),
        "target_h": C.MISSION["target_endurance_h"],
        "variant": {k: {kk: round(vv, 4) for kk, vv in v.items()}
                    for k, v in AN.variant_nose_prop().items() if k != "dmass_g"},
        "variant_dmass": AN.variant_nose_prop()["dmass_g"],
        "solar_wh": round(sim["solar_wh"]),
        "np_mac": round(st["x_np_mac"] * 100, 1),
        "cg_mac": round(st["cg_target"] * 100, 1),
        "cg_mm": round(C.WING["stations"][0][2] + st["cg_target"] * g["mac"] * 1000),
        "vh": round(st["V_h"], 3), "vv": round(st["V_v"], 4),
        "m_root": round(wb["M_root"], 1),
        "tip_defl_1g": round(wb["tip_defl_1g_mm"]),
        "swath_m": round(det["swath_m"]), "gsd_m": round(det["gsd_m"], 2),
        "coverage": round(det["coverage_sidelap_km2_h"], 1),
        "sortie_km2": round(det["coverage_sidelap_km2_h"] * 8),
        "airfoil_t": round(props["t_max"] * 100, 2),
        "airfoil_c": round(props["camber_max"] * 100, 2),
        "airfoil_r": round(props["r_min_mm"]),
        "rat_rows": rat_rows([10, 12, 15, 18, 20, 25]),
        "regen": {k: round(v, 3) for k, v in
                  AN.regen_descent(W, 20.0, 500.0).items()},
        "retraction": {k: {kk: round(vv, 4) for kk, vv in v.items()}
                       for k, v in AN.retraction_benefit().items()},
        "detect_rows": [{k: round(v, 5) for k, v in r.items()} for r in det["rows"]],
        "mast": _mast_spec(),
        "harvest": _harvest_spec(),
        "polar": [{"v": round(r["V"], 2), "cl": round(r["CL"], 2),
                   "ld": round(r["LD"], 2), "sink": round(r["sink"], 3)}
                  for r in kp["sweep"] if round(r["CL"] * 100) % 5 == 0],
        "mass_groups": sorted(
            [{"g": k, "w": round(sum(i[1] for i in v))}
             for k, v in m["groups"].items()], key=lambda d: -d["w"]),
        "cd0_rows": [{"n": n, "d": round(d, 5)} for (n, d, _t) in AN.cd0_buildup()[0]],
    }

    out = {"quant": quant, "components": comps, "rig": rig, "spec": spec,
           "bays": C.FUSELAGE["bays"]}
    path = os.path.join(ROOT, "viewer", "model.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
    tris = sum(c["nt"] for c in comps)
    print(f"wrote viewer/model.json  {os.path.getsize(path)/1024:.0f} kB, "
          f"{len(comps)} components, {tris:,} triangles")
    return path


if __name__ == "__main__":
    build()
