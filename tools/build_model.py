#!/usr/bin/env python3
"""
Generate the Forever-Flight FF-1 "Ember" 3D model from tools/config.py.

Outputs (into models/):
    ff1_deployed.stl        both pods extended -- climb / regen configuration
    ff1_stowed.stl          both pods retracted -- soaring configuration
    ff1_deployed.obj        same, with named groups for CAD import
    parts/*.stl             each component on its own, for printing
    geometry.json           parametric data the web viewer rebuilds from

The assembly STLs are visual/assembly models: components interpenetrate where
they join, so the union is not a closed manifold. Boolean the parts in Blender
or Meshmixer before slicing, or print from parts/ individually.
"""

from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import airfoil as af
import config as C
import mesh as M

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "models")
PARTS = os.path.join(OUT, "parts")


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def place_section(loop2d, chord, le_x, y, z, angle_deg, pivot_frac=0.25):
    """Put a normalised (x/c, y/c) loop into 3D at a spanwise station.

    `angle_deg` is positive nose-up and rotates about the pivot chord point.
    """
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    px = le_x + pivot_frac * chord
    out = []
    for (xc, yc) in loop2d:
        dx = (xc - pivot_frac) * chord
        dz = yc * chord
        out.append((px + dx * ca + dz * sa, y, z - dx * sa + dz * ca))
    return out


def wing_section():
    """Return (closed_loop, upper_surface, properties) for the wing aerofoil."""
    a = C.WING["airfoil"]
    if a.get("dat_file"):
        up, lo = af.load_selig_dat(os.path.join(ROOT, a["dat_file"]))
        return (af.closed_loop(up, lo, te_gap=a["te_gap"]), up,
                dict(af.section_properties(up, lo), name=a["dat_file"]))
    up, lo = af.naca4(a["camber"], a["camber_pos"], a["thickness"], n_per_side=90)
    up, r_final, iters = af.solar_flatten(up, a["solar_band"][0], a["solar_band"][1],
                                          C.WING["stations"][0][1], a["min_radius_mm"])
    props = af.section_properties(up, lo)
    props.update({"r_min_mm": r_final, "iters": iters, "name": a["name"],
                  "band": list(a["solar_band"]), "r_target_mm": a["min_radius_mm"]})
    return af.closed_loop(up, lo, te_gap=a["te_gap"]), up, props


def wing_loop():
    loop, _up, props = wing_section()
    return loop, props


def symmetric_loop(thickness, n_per_side=48, te_gap=0.006):
    up, lo = af.naca4(0.0, 0.3, thickness, n_per_side=n_per_side)
    return af.closed_loop(up, lo, te_gap=te_gap)


def wing_geometry():
    """Resolve the station table into absolute y/chord/le_x/z/twist."""
    out, z, dih_prev = [], 0.0, 0.0
    prev_y = 0.0
    for (y, chord, le_x, dih, twist) in C.WING["stations"]:
        z += (y - prev_y) * math.tan(math.radians(dih))
        out.append({"y": y, "chord": chord, "le_x": le_x,
                    "z": C.WING["root_z"] + z,
                    "angle": C.WING["incidence"] + twist})
        prev_y = y
    return out


def planform_area_mm2(stns):
    s = 0.0
    for i in range(len(stns) - 1):
        s += 0.5 * (stns[i]["chord"] + stns[i + 1]["chord"]) * (stns[i + 1]["y"] - stns[i]["y"])
    return 2.0 * s


def mean_aero_chord(stns):
    num = den = 0.0
    for i in range(len(stns) - 1):
        c0, c1 = stns[i]["chord"], stns[i + 1]["chord"]
        dy = stns[i + 1]["y"] - stns[i]["y"]
        num += dy * (c0 * c0 + c0 * c1 + c1 * c1) / 3.0
        den += dy * (c0 + c1) / 2.0
    return num / den


def densify(stns, per_bay=4):
    """Interpolate extra stations so the loft is smooth."""
    out = []
    for i in range(len(stns) - 1):
        a, b = stns[i], stns[i + 1]
        n = per_bay if i < len(stns) - 2 else per_bay + 2
        for k in range(n):
            t = k / n
            out.append({key: a[key] + t * (b[key] - a[key]) for key in a})
    out.append(stns[-1])
    return out


# --------------------------------------------------------------------------
# components
# --------------------------------------------------------------------------

def build_wing():
    loop, _up, props = wing_section()
    stns = densify(wing_geometry(), per_bay=5)
    secs = [place_section(loop, s["chord"], s["le_x"], s["y"], s["z"], s["angle"])
            for s in stns]
    # closing tip: collapse the last section onto its own camber line
    tip = secs[-1]
    cx = sum(p[0] for p in tip) / len(tip)
    cz = sum(p[2] for p in tip) / len(tip)
    secs.append([(cx + 0.25 * (p[0] - cx), tip[0][1] + 14.0, cz + 0.25 * (p[2] - cz))
                 for p in tip])
    stbd = M.loft(secs, cap_start=True, cap_end=True, name="wing_stbd")
    return stbd, props


def build_fuselage():
    stns = C.FUSELAGE["stations"]
    n = C.FUSELAGE["superellipse_n"]
    secs = []
    for (x, w, h, cz) in stns:
        ring = M.superellipse(w, h, n=n, npts=32, cz=cz)
        secs.append([(x, p[1], p[2]) for p in ring])
    return M.loft(secs, cap_start=True, cap_end=True, name="fuselage")


def _fuselage_deck(x, top=True):
    """Local top (or keel) z of the fuselage at station x, by interpolation."""
    stns = C.FUSELAGE["stations"]
    for i in range(len(stns) - 1):
        x0, w0, h0, c0 = stns[i]
        x1, w1, h1, c1 = stns[i + 1]
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            h = h0 + t * (h1 - h0)
            c = c0 + t * (c1 - c0)
            return c + (h / 2.0 if top else -h / 2.0)
    x0, w0, h0, c0 = stns[-1]
    return c0 + (h0 / 2.0 if top else -h0 / 2.0)


def build_blister(table, top=True, name="blister"):
    """Half-teardrop fairing riding on the fuselage deck or keel."""
    secs = []
    for (x, hw, ht) in table:
        base = _fuselage_deck(x, top=top)
        ring = []
        npts = 24
        for i in range(npts):
            t = 2.0 * math.pi * i / npts
            y = hw * math.cos(t)
            v = ht * math.sin(t)
            # squash the half that would sink into the fuselage
            v = v if (v > 0) == top else v * 0.18
            ring.append((x, y, base + v - (6.0 if top else -6.0)))
        secs.append(ring)
    return M.loft(secs, cap_start=True, cap_end=True, name=name)


def build_turret():
    """Faired sensor ball slung under the nose. Ellipsoid, slightly prolate."""
    t = C.FUSELAGE["turret"]
    rx, ry, rz = t["r"] * 1.18, t["r"], t["r"]
    secs = []
    steps = 18
    for i in range(steps + 1):
        a = math.pi * (i / steps)
        x = -rx * math.cos(a)
        rr = math.sin(a)
        secs.append([(t["x"] + x, ry * rr * math.cos(2 * math.pi * k / 20),
                      t["z"] + rz * rr * math.sin(2 * math.pi * k / 20))
                     for k in range(20)])
    return M.loft(secs, cap_start=True, cap_end=True, name="sensor_turret")


def build_vtail():
    v = C.VTAIL
    loop = symmetric_loop(v["thickness"])
    secs = []
    steps = 8
    for i in range(steps + 1):
        t = i / steps
        chord = v["root_chord"] + t * (v["tip_chord"] - v["root_chord"])
        span = t * v["panel_span"]
        le_x = v["le_x"] + span * math.tan(math.radians(v["sweep_le"]))
        secs.append(place_section(loop, chord, le_x, span, v["root_z"], v["incidence"]))
    panel = M.loft(secs, cap_start=True, cap_end=True, name="vtail_stbd")
    # rotate the panel up about the boom axis to make the V
    panel.apply(M.rot_about(v["dihedral"], "x", (0.0, 0.0, v["root_z"])))
    return panel


def build_strut(pivot, length, chord, thickness, angle_deg, name):
    """A symmetric-section strut from `pivot`, at `angle_deg` in the XZ plane."""
    loop = symmetric_loop(thickness, n_per_side=32)
    steps = 6
    # section in the XY plane, extruded along local +Z
    secs = []
    for i in range(steps + 1):
        t = i / steps
        c = chord * (1.0 - 0.18 * t)
        secs.append([((xc - 0.30) * c, yc * c, t * length) for (xc, yc) in loop])
    m = M.loft(secs, cap_start=True, cap_end=True, name=name)
    # local +Z -> the requested direction in the XZ plane
    m.apply(M.rot_y(90.0 - angle_deg))
    m.apply(M.translate(*pivot))
    return m


def build_nacelle(length, dia, spinner_len, spinner_dia, name):
    secs = []
    prof = [(-length * 0.5, dia * 0.42), (-length * 0.30, dia * 0.50),
            (length * 0.15, dia * 0.50), (length * 0.42, dia * 0.44),
            (length * 0.50, dia * 0.30)]
    for (x, r) in prof:
        secs.append([(x, p[1], p[2]) for p in M.circle(r, npts=20)])
    body = M.loft(secs, cap_start=True, cap_end=True, name=name)
    sp = []
    for i in range(9):
        t = i / 8.0
        r = (spinner_dia / 2.0) * math.sqrt(max(1e-6, 1.0 - t * t))
        sp.append([(length * 0.5 + t * spinner_len, p[1], p[2])
                   for p in M.circle(r, npts=20)])
    body.merge(M.loft(sp, cap_start=True, cap_end=False, name=name + "_spinner"),
               group_name=name + "_spinner")
    return body


def build_blade(r0, r1, chord_at, pitch_at, thickness, camber, name, steps=10):
    """Radial blade along +Y, chord in the XZ plane, twisted about +Y."""
    up, lo = af.naca4(camber, 0.40, thickness, n_per_side=26)
    loop = af.closed_loop(up, lo, te_gap=0.01)
    secs = []
    for i in range(steps + 1):
        t = i / steps
        r = r0 + t * (r1 - r0)
        c = chord_at(r)
        beta = math.radians(pitch_at(r))
        cb, sb = math.cos(beta), math.sin(beta)
        ring = []
        for (xc, yc) in loop:
            dx = (xc - 0.30) * c
            dz = yc * c
            ring.append((dx * cb - dz * sb, r, dx * sb + dz * cb))
        secs.append(ring)
    return M.loft(secs, cap_start=True, cap_end=True, name=name)


def build_propeller(deployed=True):
    m = C.MOTOR
    R = m["prop_dia"] / 2.0
    pitch_mm = m["prop_pitch_in"] * 25.4

    def chord(r):
        u = r / R
        return 26.0 * (1.0 - 0.55 * (u - 0.45) ** 2 / 0.30) * (1.0 - 0.88 * max(0.0, u - 0.86) / 0.14)

    def beta(r):
        return math.degrees(math.atan2(pitch_mm, 2.0 * math.pi * max(r, 12.0)))

    prop = M.Mesh("propeller")
    for b in range(m["blades"]):
        blade = build_blade(20.0, R, chord, beta, 0.10, 0.045, f"blade_{b}")
        if deployed:
            blade.apply(M.rot_x(360.0 * b / m["blades"] + 78.0))
        else:                                   # folded back along the nacelle
            blade.apply(M.rot_z(-78.0))
            blade.apply(M.rot_x(180.0 * b + 90.0))
        prop.merge(blade, group_name=f"prop_blade_{b}")
    return prop


def build_turbine(deployed=True):
    r = C.RAT
    R = r["rotor_dia"] / 2.0
    lam = r["lambda_design"]

    def chord(rr):
        u = rr / R
        return r["blade_chord"] * (1.25 - 0.55 * u) * (1.0 - 0.85 * max(0.0, u - 0.90) / 0.10)

    def beta(rr):
        u = max(rr / R, 0.20)
        phi = math.degrees(math.atan2(2.0, 3.0 * lam * u))   # optimum inflow angle
        return max(phi - 5.0, 3.0)                            # minus design alpha

    rot = M.Mesh("turbine")
    for b in range(r["blades"]):
        blade = build_blade(r["hub_r"], R, chord, beta, 0.09, 0.035, f"tblade_{b}")
        if deployed:
            blade.apply(M.rot_x(360.0 * b / r["blades"] + 20.0))
        else:
            blade.apply(M.rot_z(74.0))
            blade.apply(M.rot_x(360.0 * b / r["blades"] + 20.0))
        rot.merge(blade, group_name=f"turbine_blade_{b}")
    return rot


def build_motor_pod(deployed=True):
    m = C.MOTOR
    ang = m["deployed_angle"] if deployed else m["stowed_angle"]
    pod = M.Mesh("motor_pod")
    pod.merge(build_strut(m["pivot"], m["pylon_len"], m["pylon_chord"],
                          m["pylon_thickness"], ang, "motor_pylon"),
              group_name="motor_pylon")

    tipx = m["pivot"][0] + m["pylon_len"] * math.cos(math.radians(ang))
    tipz = m["pivot"][2] + m["pylon_len"] * math.sin(math.radians(ang))
    hub = (tipx + m["nacelle_offset"], 0.0, tipz)

    # Pusher: the nacelle nose faces into the flow, the spinner and prop disc
    # sit at the aft end, so build_nacelle's native orientation is correct.
    nac = build_nacelle(m["nacelle_len"], m["nacelle_dia"],
                        m["spinner_len"], m["spinner_dia"], "motor_nacelle")
    nac.apply(M.rot_y(-m["thrust_angle"]))
    nac.apply(M.translate(*hub))
    pod.merge(nac, group_name="motor_nacelle")

    prop = build_propeller(deployed=deployed)
    prop.apply(M.rot_y(-m["thrust_angle"]))
    prop.apply(M.translate(hub[0] + m["nacelle_len"] * 0.5, 0.0, hub[2]))
    pod.merge(prop, group_name="propeller")
    return pod, hub


def build_rat_pod(deployed=True):
    r = C.RAT
    ang = r["deployed_angle"] if deployed else r["stowed_angle"]
    pod = M.Mesh("rat_pod")
    pod.merge(build_strut(r["pivot"], r["arm_len"], r["arm_chord"],
                          r["arm_thickness"], ang, "rat_arm"),
              group_name="rat_arm")

    hubx = r["pivot"][0] + r["arm_len"] * math.cos(math.radians(ang))
    hubz = r["pivot"][2] + r["arm_len"] * math.sin(math.radians(ang))
    hub = (hubx, 0.0, hubz)

    # Tractor-style turbine: rotor upstream, generator body trailing aft.
    nac = build_nacelle(r["nacelle_len"], r["nacelle_dia"],
                        r["spinner_len"], r["spinner_dia"], "rat_nacelle")
    nac.apply(M.rot_y(180.0))
    nac.apply(M.translate(*hub))
    pod.merge(nac, group_name="rat_nacelle")

    rot = build_turbine(deployed=deployed)
    rot.apply(M.translate(hub[0] - r["nacelle_len"] * 0.5 - 4.0, 0.0, hub[2]))
    pod.merge(rot, group_name="turbine_rotor")
    return pod, hub


# --------------------------------------------------------------------------
# solar array layout
# --------------------------------------------------------------------------

def solar_layout():
    """Place cell strips on the wing upper surface. Returns strips + totals."""
    s, w = C.SOLAR, C.WING
    stns = wing_geometry()

    def chord_at(y):
        for i in range(len(stns) - 1):
            if stns[i]["y"] <= y <= stns[i + 1]["y"]:
                t = (y - stns[i]["y"]) / (stns[i + 1]["y"] - stns[i]["y"])
                return stns[i]["chord"] + t * (stns[i + 1]["chord"] - stns[i]["chord"])
        return stns[-1]["chord"]

    strips = []
    y = s["centre_gap_mm"] / 2.0
    half_span = w["span"] / 2.0 - s["tip_margin_mm"]
    while y + s["strip_l_mm"] <= half_span:
        c = chord_at(y + s["strip_l_mm"] / 2.0)
        for k in range(s["n_strips_chordwise"]):
            x0 = s["chord_start"] * c + k * s["strip_w_mm"]
            if x0 + s["strip_w_mm"] > w["airfoil"]["solar_band"][1] * c:
                continue
            strips.append({"y0": y, "y1": y + s["strip_l_mm"],
                           "xc0": x0 / c, "xc1": (x0 + s["strip_w_mm"]) / c,
                           "chord": c})
        y += s["strip_l_mm"] + 2.0          # 2 mm inter-cell gap
    n_wing_side = len(strips)

    # V-tail sub-array. Cosine losses are worse at 38 deg dihedral, so this is
    # accounted separately in the energy model rather than lumped in.
    v = C.VTAIL
    tail = []
    ty = 20.0
    while ty + s["strip_l_mm"] <= v["panel_span"] - 20.0:
        t = (ty + s["strip_l_mm"] / 2.0) / v["panel_span"]
        c = v["root_chord"] + t * (v["tip_chord"] - v["root_chord"])
        for k in range(2):
            x0 = 0.20 * c + k * s["strip_w_mm"]
            if x0 + s["strip_w_mm"] > 0.86 * c:
                continue
            tail.append({"y0": ty, "y1": ty + s["strip_l_mm"],
                         "xc0": x0 / c, "xc1": (x0 + s["strip_w_mm"]) / c,
                         "chord": c})
        ty += s["strip_l_mm"] + 2.0
    n_tail_side = len(tail)

    strip_m2 = (s["strip_l_mm"] / 1000.0) * (s["strip_w_mm"] / 1000.0)
    wing_total, tail_total = 2 * n_wing_side, 2 * n_tail_side
    wing_area, tail_area = wing_total * strip_m2, tail_total * strip_m2
    return {"strips": strips, "tail_strips": tail,
            "n_strips_wing": wing_total, "n_strips_tail": tail_total,
            "n_strips_total": wing_total + tail_total,
            "n_cells_equiv": (wing_total + tail_total) / s["strips_per_cell"],
            "wing_area_m2": wing_area, "tail_area_m2": tail_area,
            "active_area_m2": wing_area + tail_area,
            "p_stc_wing_w": wing_area * 1000.0 * s["cell_eff_stc"],
            "p_stc_tail_w": tail_area * 1000.0 * s["cell_eff_stc"],
            "p_stc_w": (wing_area + tail_area) * 1000.0 * s["cell_eff_stc"]}


def _interp_station(stns, y):
    y = min(max(y, stns[0]["y"]), stns[-1]["y"])
    for i in range(len(stns) - 1):
        if stns[i]["y"] <= y <= stns[i + 1]["y"]:
            a, b = stns[i], stns[i + 1]
            t = (y - a["y"]) / (b["y"] - a["y"]) if b["y"] > a["y"] else 0.0
            return {k: a[k] + t * (b[k] - a[k]) for k in a}
    return dict(stns[-1])


def _upper_yc(upper, xc):
    """Upper-surface y/c at a chord fraction, by linear interpolation."""
    for i in range(len(upper) - 1):
        if upper[i][0] <= xc <= upper[i + 1][0]:
            x0, y0 = upper[i]
            x1, y1 = upper[i + 1]
            t = (xc - x0) / (x1 - x0) if x1 > x0 else 0.0
            return y0 + t * (y1 - y0)
    return upper[-1][1]


def build_solar_cells(standoff=0.7):
    """Thin cell strips conforming to the wing upper surface, one group each."""
    _loop, upper, _p = wing_section()
    stns = wing_geometry()
    layout = solar_layout()
    out = M.Mesh("solar_array")

    def surf(y, xc):
        st = _interp_station(stns, abs(y))
        pt = place_section([(xc, _upper_yc(upper, xc))], st["chord"], st["le_x"],
                           y, st["z"], st["angle"])[0]
        return (pt[0], pt[1], pt[2] + standoff)

    for si, strip in enumerate(layout["strips"]):
        for sign in (1.0, -1.0):
            nx, nc = 3, 5
            grid = []
            for i in range(nx + 1):
                ty = strip["y0"] + (strip["y1"] - strip["y0"]) * i / nx
                row = [surf(sign * ty, strip["xc0"]
                            + (strip["xc1"] - strip["xc0"]) * k / nc)
                       for k in range(nc + 1)]
                grid.append(row)
            cell = M.Mesh(f"cell_{si}")
            idx = [[cell.add_vertex(p) for p in row] for row in grid]
            for i in range(nx):
                for k in range(nc):
                    if sign > 0:
                        cell.quad(idx[i][k], idx[i][k + 1], idx[i + 1][k + 1], idx[i + 1][k])
                    else:
                        cell.quad(idx[i][k], idx[i + 1][k], idx[i + 1][k + 1], idx[i][k + 1])
            out.merge(cell, group_name="solar_cells")
    return out


# --------------------------------------------------------------------------
# assembly
# --------------------------------------------------------------------------

def assemble(deployed=True):
    a = M.Mesh("ff1")
    wing, props = build_wing()
    a.merge(wing, group_name="wing_stbd")
    a.merge(wing.mirrored_y(), group_name="wing_port")
    a.merge(build_fuselage(), group_name="fuselage")
    a.merge(build_blister(C.FUSELAGE["dorsal_spine"], True, "dorsal_spine"),
            group_name="dorsal_spine")
    a.merge(build_blister(C.FUSELAGE["ventral_fairing"], False, "ventral_fairing"),
            group_name="ventral_fairing")
    a.merge(build_turret(), group_name="sensor_turret")
    vt = build_vtail()
    a.merge(vt, group_name="vtail_stbd")
    a.merge(vt.mirrored_y(), group_name="vtail_port")
    mp, mhub = build_motor_pod(deployed)
    a.merge(mp, group_name="motor_pod")
    rp, rhub = build_rat_pod(deployed)
    a.merge(rp, group_name="rat_pod")
    a.merge(build_solar_cells(), group_name="solar_cells")
    return a, props, mhub, rhub


def main():
    os.makedirs(PARTS, exist_ok=True)
    stns = wing_geometry()
    S_mm2 = planform_area_mm2(stns)
    mac = mean_aero_chord(stns)
    solar = solar_layout()

    dep, props, mhub, rhub = assemble(True)
    stow, _, mhub_s, rhub_s = assemble(False)

    M.write_stl(dep, os.path.join(OUT, "ff1_deployed.stl"), "FF-1 Ember, pods deployed")
    M.write_stl(stow, os.path.join(OUT, "ff1_stowed.stl"), "FF-1 Ember, pods stowed")
    M.write_obj(dep, os.path.join(OUT, "ff1_deployed.obj"), "FF-1_Ember_deployed")
    M.write_obj(stow, os.path.join(OUT, "ff1_stowed.obj"), "FF-1_Ember_stowed")

    wing, _ = build_wing()
    for name, m in [
                    ("solar_cells", build_solar_cells()),("wing_starboard", wing),
                    ("wing_port", wing.mirrored_y()),
                    ("fuselage", build_fuselage()),
                    ("dorsal_spine", build_blister(C.FUSELAGE["dorsal_spine"], True, "s")),
                    ("ventral_fairing", build_blister(C.FUSELAGE["ventral_fairing"], False, "v")),
                    ("sensor_turret", build_turret()),
                    ("vtail_starboard", build_vtail()),
                    ("vtail_port", build_vtail().mirrored_y()),
                    ("motor_pod_deployed", build_motor_pod(True)[0]),
                    ("rat_pod_deployed", build_rat_pod(True)[0])]:
        M.write_stl(m, os.path.join(PARTS, f"{name}.stl"), f"FF-1 {name}")

    bb_lo, bb_hi = dep.bounds()
    geo = {
        "name": "Forever-Flight FF-1 Ember",
        "units": "mm",
        "axes": "+X aft, +Y starboard, +Z up",
        "wing": {"span_mm": C.WING["span"],
                 "area_m2": S_mm2 / 1e6,
                 "aspect_ratio": (C.WING["span"] ** 2) / S_mm2,
                 "mac_mm": mac,
                 "root_chord_mm": stns[0]["chord"],
                 "tip_chord_mm": stns[-1]["chord"],
                 "stations": stns,
                 "airfoil": props},
        "solar": {k: v for k, v in solar.items()
                  if k not in ("strips", "tail_strips")},
        "solar_strips": solar["strips"],
        "solar_tail_strips": solar["tail_strips"],
        "fuselage": C.FUSELAGE["stations"],
        "dorsal_spine": C.FUSELAGE["dorsal_spine"],
        "ventral_fairing": C.FUSELAGE["ventral_fairing"],
        "vtail": C.VTAIL,
        "motor": dict(C.MOTOR, hub_deployed=mhub, hub_stowed=mhub_s),
        "rat": dict(C.RAT, hub_deployed=rhub, hub_stowed=rhub_s),
        "bays": C.FUSELAGE["bays"],
        "bbox": {"min": bb_lo, "max": bb_hi},
        "mesh": {"vertices": len(dep.v), "triangles": len(dep.f)},
    }
    with open(os.path.join(OUT, "geometry.json"), "w") as fh:
        json.dump(geo, fh, indent=1)

    print(f"wing area      {S_mm2/1e6:.4f} m^2")
    print(f"aspect ratio   {(C.WING['span']**2)/S_mm2:.2f}")
    print(f"MAC            {mac:.1f} mm")
    print(f"aerofoil       t/c {props['t_max']*100:.2f}% @ {props['t_at']*100:.1f}%c, "
          f"camber {props['camber_max']*100:.2f}% @ {props['camber_at']*100:.1f}%c, "
          f"min upper R {props['r_min_mm']:.0f} mm")
    print(f"solar          wing {solar['n_strips_wing']} + tail "
          f"{solar['n_strips_tail']} strips = {solar['n_cells_equiv']:.2f} cells, "
          f"{solar['active_area_m2']*1e4:.0f} cm^2, {solar['p_stc_w']:.1f} W STC "
          f"(wing {solar['p_stc_wing_w']:.1f} + tail {solar['p_stc_tail_w']:.1f})")
    print(f"bbox           x {bb_lo[0]:.0f}..{bb_hi[0]:.0f}  "
          f"y {bb_lo[1]:.0f}..{bb_hi[1]:.0f}  z {bb_lo[2]:.0f}..{bb_hi[2]:.0f} mm")
    print(f"deployed mesh  {len(dep.v)} verts, {len(dep.f)} tris")
    print(f"stowed mesh    {len(stow.v)} verts, {len(stow.f)} tris")
    print(f"wing manifold  edge histogram {wing.edge_report()}")


if __name__ == "__main__":
    main()
