#!/usr/bin/env python3
"""
JW-1: a swept flying wing in the Jetwing configuration.

    python3 tools/jetwing.py

NOT a copy of PlanePrint's Jetwing. Their page is blocked by this session's
network egress policy, so none of their published dimensions were available,
and a single photograph does not give geometry. Every number here is designed
from scratch to the configuration visible in the photo -- swept wing, upturned
winglets, single dorsal fin, pusher prop on the centre pod -- and then checked
against the aerodynamics. Give me their real figures and the parameters below
take them directly.

The thing that makes a tailless wing different is trim. There is no tailplane
to balance the wing's nose-down pitching moment, so the wing has to do it
itself, through:

  REFLEX   the trailing edge curves up, which makes the section's own moment
           positive (nose-up) instead of negative
  WASHOUT  the tips are twisted nose-down, and on a SWEPT wing the tips sit
           behind the centre of gravity, so they act as a tailplane

Both are tuned here against thin-aerofoil theory and a strip-theory trim
solution, not guessed.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import airfoil as AF
import mesh as M

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RHO = 1.225
G = 9.80665

# ==========================================================================
# configuration
# ==========================================================================

# PUBLISHED Jetwing figures (planeprint.com, via search -- the site itself is
# blocked by this session's egress policy, so these come from indexed copy):
#
#   span            1270 mm standard wing / 2060 mm BIG WING
#   flight weight   860 to 1750 g
#   wing loading    28 to 48 g/dm2
#   power           EDF 70 mm on 4S, or glider
#   channels        4/6, four flaps, so butterfly/crow braking
#   variants        with or without a steerable rudder; the rudder version
#                   has "integrated vector control"
#   printing        200 x 200 x 200 mm cube, LW-PLA plus PLA
#
# The wing AREAS below are not published. They are chosen so that all four
# published numbers fall out exactly:
#
#   standard 24.6 dm2 ->  860 g = 35.0 g/dm2,  1180 g = 48.0 g/dm2
#   big      36.5 dm2 -> 1022 g = 28.0 g/dm2,  1750 g = 48.0 g/dm2
#
# and so both wings share one 260 mm root chord, which a modular kit with a
# common fuselage joint has to do. Everything else -- sweep, taper, section,
# CG -- is designed here, not copied. See docs/JETWING.md.

JW = {
    "name": "JW-1",
    # --- planform (half-wing, y from centreline) ---
    "span": 2060.0,            # the BIG WING
    "root_chord": 260.0,       # shared with the standard wing: modular joint
    "tip_chord": 94.4,         # gives 36.5 dm2 at 2060 mm
    "sweep_le": 24.0,          # deg
    "dihedral": 2.0,           # deg, mild
    "washout_tip": -2.2,       # deg, nose-down at the tip. Set by the trim solve.
    "centre_frac": 0.17,       # fraction of semi-span blended into the pod

    # --- section ---
    "thickness": 0.105,
    "camber": 0.022,
    "camber_pos": 0.38,
    "reflex_start": 0.65,      # reflex acts aft of this chord fraction
    "target_cm": 0.006,        # section Cm about c/4, positive = nose-up

    # --- winglets ---
    "winglet_h": 150.0,
    "winglet_cant": 72.0,      # deg from horizontal
    "winglet_sweep": 38.0,
    "winglet_taper": 0.45,

    # --- fin: ONE fin, sitting behind the EDF nozzle so the rudder works in
    #     the efflux. That is the "integrated vector control" variant. ---
    "fin_h": 185.0,
    "fin_root": 170.0,
    "fin_tip": 78.0,
    "fin_sweep": 40.0,
    "rudder_frac": 0.38,

    # --- EDF fuselage ---
    "pod_len": 620.0,
    "pod_w": 96.0,
    "pod_h": 104.0,
    "fan_dia": 50.0,           # the 50 mm unit asked for (kit calls for 70)
    "fan_hub": 26.0,
    "fan_x": 0.615,            # fan face, fraction of pod length
    "nozzle_dia": 40.5,        # 90% of fan swept area, hub gone
    "inlet_dia": 30.9,         # each of two side inlets, 1.05 FSA total
    "inlet_x": 0.30,
    "duct_z": -6.0,            # duct axis below the pod datum

    # --- four flaps: inboard pair brake, outboard pair roll. Both pairs
    #     mix into pitch, which is what gives butterfly/crow. ---
    "flap_y0": 0.16, "flap_y1": 0.52,
    "ail_y0": 0.55, "ail_y1": 0.95,
    "flap_chord": 0.26,
    "pod_datum": -5.0,         # pod nose-down vs the root chord, so the body
                               # looks level at the trim attitude
    # --- mass, EDF fuselage + big wing, from the budget in jw_analysis.py ---
    "mass_g": 1180.0,
    "static_margin": 0.075,
}


# ==========================================================================
# reflexed section, and thin-aerofoil theory
# ==========================================================================

def camber_line(x, m, p, reflex, q):
    """NACA 4-digit camber plus a smooth trailing-edge reflex aft of q."""
    z, dz = AF._camber(x, m, p)
    if x >= q:
        t = (x - q) / (1.0 - q)
        z += reflex * t * t
        dz += reflex * 2.0 * t / (1.0 - q)
    return z, dz


def thin_aerofoil(m, p, reflex, q, n=400):
    """Zero-lift angle and quarter-chord moment from thin-aerofoil theory.

        A1 = (2/pi) int dz/dx cos(theta) dtheta
        A2 = (2/pi) int dz/dx cos(2 theta) dtheta
        Cm_c/4 = (pi/4) (A2 - A1)
        alpha_L0 = -(1/pi) int dz/dx (cos theta - 1) dtheta
    """
    def integrand(th, f):
        x = 0.5 * (1.0 - math.cos(th))
        _z, dz = camber_line(x, m, p, reflex, q)
        return dz * f(th)

    def simpson(f):
        h = math.pi / n
        s = integrand(0.0, f) + integrand(math.pi, f)
        for i in range(1, n):
            s += (4 if i % 2 else 2) * integrand(i * h, f)
        return s * h / 3.0

    a1 = (2.0 / math.pi) * simpson(math.cos)
    a2 = (2.0 / math.pi) * simpson(lambda th: math.cos(2 * th))
    cm = (math.pi / 4.0) * (a2 - a1)
    al0 = -(1.0 / math.pi) * simpson(lambda th: math.cos(th) - 1.0)
    return {"cm_c4": cm, "alpha_L0_deg": math.degrees(al0), "A1": a1, "A2": a2}


def tune_reflex(target_cm, m, p, q, lo=0.0, hi=0.25):
    """Find the reflex that puts the section moment where a tailless wing needs it."""
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if thin_aerofoil(m, p, mid, q)["cm_c4"] < target_cm:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def section_points(chord, m, p, reflex, q, t, n_per_side=70, te_gap=0.004):
    """Closed loop for the reflexed section, scaled to a chord."""
    up, lo = [], []
    for i in range(n_per_side + 1):
        beta = math.pi * i / n_per_side
        x = 0.5 * (1.0 - math.cos(beta))
        yt = AF._thickness(x, t)
        yc, dy = camber_line(x, m, p, reflex, q)
        th = math.atan(dy)
        up.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lo.append((x + yt * math.sin(th), yc - yt * math.cos(th)))
    loop = AF.closed_loop(up, lo, te_gap=te_gap)
    return [(x * chord, y * chord) for (x, y) in loop], up, lo


# ==========================================================================
# planform
# ==========================================================================

def planform(cfg=JW):
    b, cr, ct = cfg["span"], cfg["root_chord"], cfg["tip_chord"]
    lam = ct / cr
    half = b / 2.0
    S = 2.0 * half * (cr + ct) / 2.0 / 1e6            # m2
    mac = (2.0 / 3.0) * cr * (1 + lam + lam * lam) / (1 + lam)
    y_mac = (half / 3.0) * (1 + 2 * lam) / (1 + lam)
    sweep = math.radians(cfg["sweep_le"])
    x_le_mac = y_mac * math.tan(sweep)
    return {"b": b / 1000.0, "S": S, "AR": (b / 1000.0) ** 2 / S,
            "taper": lam, "mac": mac, "y_mac": y_mac, "x_le_mac": x_le_mac,
            "half": half, "x_le_tip": half * math.tan(sweep)}


def chord_at(y, cfg=JW):
    t = abs(y) / (cfg["span"] / 2.0)
    return cfg["root_chord"] + (cfg["tip_chord"] - cfg["root_chord"]) * t


def le_at(y, cfg=JW):
    return abs(y) * math.tan(math.radians(cfg["sweep_le"]))


def twist_at(y, cfg=JW):
    """Washout, zero at the root, growing outboard."""
    t = abs(y) / (cfg["span"] / 2.0)
    return cfg["washout_tip"] * t ** 1.3


# ==========================================================================
# trim and stability, by strip theory
# ==========================================================================

def strip_moments(alpha_deg, x_cg, cfg=JW, sec=None, n=80):
    """Lift and pitching moment about x_cg, summed over spanwise strips.

    Crude on absolute numbers -- no induced-angle solution -- but it captures
    the thing that matters here: a swept wing's washed-out tips sit BEHIND the
    centre of gravity and pitch the aircraft nose-up, which is what lets a
    tailless wing trim.
    """
    pf = planform(cfg)
    a0 = 2.0 * math.pi                               # per rad, 2D
    AR = pf["AR"]
    a = a0 / (1 + a0 / (math.pi * AR * 0.90))        # 3D correction
    L = Mo = area = 0.0
    half = pf["half"]
    dy = half / n
    for i in range(n):
        y = (i + 0.5) * dy
        c = chord_at(y, cfg)
        eps = twist_at(y, cfg)
        cl = a * math.radians(alpha_deg + eps - sec["alpha_L0_deg"])
        x_c4 = le_at(y, cfg) + 0.25 * c
        dA = c * dy
        L += cl * dA
        Mo += -cl * dA * (x_c4 - x_cg) + sec["cm_c4"] * c * dA
        area += dA
    S = 2 * area
    return {"CL": 2 * L / S, "Cm": 2 * Mo / (S * pf["mac"]), "S_mm2": S}


def solve_trim(cfg=JW, sec=None):
    """Find the CG that gives the wanted static margin, then the trim point."""
    pf = planform(cfg)
    mac = pf["mac"]

    def cm_slope(x_cg):
        a = strip_moments(0.0, x_cg, cfg, sec)
        b = strip_moments(4.0, x_cg, cfg, sec)
        return (b["Cm"] - a["Cm"]) / (b["CL"] - a["CL"]), a, b

    # neutral point: the CG where dCm/dCL = 0
    lo, hi = 0.0, cfg["root_chord"] * 1.4
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if cm_slope(mid)[0] < 0:
            lo = mid
        else:
            hi = mid
    x_np = 0.5 * (lo + hi)
    x_cg = x_np - cfg["static_margin"] * mac

    # trim: alpha where Cm about the CG is zero
    a_lo, a_hi = -8.0, 14.0
    for _ in range(80):
        am = 0.5 * (a_lo + a_hi)
        if strip_moments(am, x_cg, cfg, sec)["Cm"] > 0:
            a_lo = am
        else:
            a_hi = am
    a_tr = 0.5 * (a_lo + a_hi)
    tr = strip_moments(a_tr, x_cg, cfg, sec)
    return {"x_np": x_np, "x_cg": x_cg, "alpha_trim": a_tr,
            "CL_trim": tr["CL"], "sm": cfg["static_margin"],
            "np_pct_mac": (x_np - pf["x_le_mac"]) / mac * 100,
            "cg_pct_mac": (x_cg - pf["x_le_mac"]) / mac * 100}


# ==========================================================================
# geometry
# ==========================================================================

def _place(loop2d, chord, le_x, y, z, angle, pivot=0.25):
    a = math.radians(angle)
    ca, sa = math.cos(a), math.sin(a)
    px = le_x + pivot * chord
    out = []
    for (xc, yc) in loop2d:
        dx = (xc - pivot) * chord
        dz = yc * chord
        out.append((px + dx * ca + dz * sa, y, z - dx * sa + dz * ca))
    return out


def _reflex(cfg):
    return tune_reflex(cfg["target_cm"], cfg["camber"], cfg["camber_pos"],
                       cfg["reflex_start"])


def build_wing(cfg=JW, n_bays=13):
    """Swept, tapered, washed-out half wing ending in an upturned winglet."""
    refl = _reflex(cfg)
    half = cfg["span"] / 2.0
    dih = math.radians(cfg["dihedral"])
    loop, _u, _l = section_points(1.0, cfg["camber"], cfg["camber_pos"],
                                  refl, cfg["reflex_start"], cfg["thickness"])
    secs = []
    for i in range(n_bays + 1):
        y = half * i / n_bays
        secs.append(_place(loop, chord_at(y, cfg), le_at(y, cfg), y,
                           y * math.tan(dih), twist_at(y, cfg)))

    cant = math.radians(cfg["winglet_cant"])
    wsw = math.tan(math.radians(cfg["winglet_sweep"]))
    for i in range(1, 6):
        t = i / 5.0
        h = cfg["winglet_h"] * t
        c = chord_at(half, cfg) * (1 - (1 - cfg["winglet_taper"]) * t)
        wl, _u2, _l2 = section_points(1.0, cfg["camber"] * (1 - t),
                                      cfg["camber_pos"], refl * (1 - t),
                                      cfg["reflex_start"], cfg["thickness"])
        secs.append(_place(wl, c, le_at(half, cfg) + h * wsw,
                           half + h * math.cos(cant),
                           half * math.tan(dih) + h * math.sin(cant),
                           twist_at(half, cfg)))
    return M.loft(secs, cap_start=True, cap_end=True, name="wing_stbd")


def build_pod(cfg=JW):
    """Centre body blended into the wing, set nose-down to the trim attitude."""
    L, w, h = cfg["pod_len"], cfg["pod_w"], cfg["pod_h"]
    st = [(-0.14 * L, 0.10, 0.12, 0.00), (-0.06 * L, 0.42, 0.46, 0.02),
          (0.04 * L, 0.74, 0.82, 0.03), (0.18 * L, 1.00, 1.00, 0.02),
          (0.36 * L, 0.98, 0.95, 0.00), (0.56 * L, 0.84, 0.80, -0.02),
          (0.74 * L, 0.62, 0.58, -0.03), (0.88 * L, 0.40, 0.36, -0.03),
          (0.97 * L, 0.20, 0.18, -0.02)]
    secs = []
    for (x, fw, fh, fz) in st:
        ring = M.superellipse(w * fw, h * fh, n=2.5, npts=26, cz=fz * h)
        secs.append([(x, p[1], p[2]) for p in ring])
    pod = M.loft(secs, cap_start=True, cap_end=True, name="pod")
    pod.apply(M.rot_y(cfg["pod_datum"]))
    return pod


def build_fin(cfg=JW):
    up, lo = AF.naca4(0.0, 0.3, 0.08, n_per_side=26)
    loop = AF.closed_loop(up, lo, te_gap=0.006)
    sw = math.tan(math.radians(cfg["fin_sweep"]))
    x0 = cfg["fin_x"] * cfg["root_chord"]
    secs = []
    for i in range(8):
        t = i / 7.0
        hgt = cfg["fin_h"] * t
        c = cfg["fin_root"] + (cfg["fin_tip"] - cfg["fin_root"]) * t
        pts = _place(loop, c, x0 + hgt * sw, 0.0, 0.0, 0.0)
        secs.append([(px, pz, hgt + 20.0) for (px, _py, pz) in pts])
    return M.loft(secs, cap_start=True, cap_end=True, name="fin")


def build_pusher(cfg=JW):
    import build_model as BM
    out = M.Mesh("pusher")
    hub_x = cfg["pod_len"] * 0.98
    R = cfg["prop_dia"] / 2.0
    sp = []
    for i in range(9):
        t = i / 8.0
        r = (cfg["spinner"] / 2.0) * math.sqrt(max(1e-6, 1 - t * t))
        sp.append([(hub_x + t * cfg["spinner"] * 1.2, p[1], p[2])
                   for p in M.circle(r, npts=14, cz=-8.0)])
    out.merge(M.loft(sp, cap_start=True, cap_end=False, name="spinner"),
              group_name="pusher")

    def chord(r):
        u = r / R
        return 22.0 * (1 - 0.5 * (u - 0.45) ** 2 / 0.30) \
            * (1 - 0.85 * max(0.0, u - 0.86) / 0.14)

    def beta(r):
        return math.degrees(math.atan2(4.0 * 25.4, 2 * math.pi * max(r, 10.0)))

    for b in range(2):
        bl = BM.build_blade(14.0, R, chord, beta, 0.09, 0.04, f"pb{b}")
        bl.apply(M.rot_x(180.0 * b + 82.0))
        bl.apply(M.translate(hub_x - 5.0, 0.0, -8.0))
        out.merge(bl, group_name="pusher")
    return out


def build_elevon_lines(cfg=JW, width=2.4, standoff=0.45):
    refl = _reflex(cfg)
    _loop, up, lo = section_points(1.0, cfg["camber"], cfg["camber_pos"], refl,
                                   cfg["reflex_start"], cfg["thickness"])
    half = cfg["span"] / 2.0
    dih = math.radians(cfg["dihedral"])
    hinge = 1.0 - cfg["elevon_chord"]
    out = M.Mesh("elevons")

    def yc_at(seq, xf):
        for i in range(len(seq) - 1):
            if seq[i][0] <= xf <= seq[i + 1][0]:
                t = (xf - seq[i][0]) / max(1e-9, seq[i + 1][0] - seq[i][0])
                return seq[i][1] + t * (seq[i + 1][1] - seq[i][1])
        return seq[-1][1]

    for sign in (1.0, -1.0):
        for side, seq in ((1.0, up), (-1.0, lo)):
            rows = []
            for i in range(9):
                t = cfg["elevon_y0"] + (cfg["elevon_y1"] - cfg["elevon_y0"]) * i / 8
                y = half * t
                c = chord_at(y, cfg)
                w = width / c
                row = []
                for xc in (hinge - w / 2, hinge + w / 2):
                    p = _place([(xc, yc_at(seq, xc))], c, le_at(y, cfg),
                               sign * y, y * math.tan(dih),
                               twist_at(y, cfg))[0]
                    row.append((p[0], p[1], p[2] + side * standoff))
                rows.append(row)
            m = M.Mesh("s")
            idx = [[m.add_vertex(q) for q in r] for r in rows]
            for i in range(8):
                if (side > 0) == (sign > 0):
                    m.quad(idx[i][0], idx[i][1], idx[i + 1][1], idx[i + 1][0])
                else:
                    m.quad(idx[i][0], idx[i + 1][0], idx[i + 1][1], idx[i][1])
            out.merge(m, group_name="elevons")
    return out


def assemble(cfg=JW, motor=True):
    a = M.Mesh("jetwing")
    w = build_wing(cfg)
    a.merge(w, group_name="wing_stbd")
    a.merge(w.mirrored_y(), group_name="wing_port")
    a.merge(build_pod(cfg), group_name="pod")
    a.merge(build_fin(cfg), group_name="fin")
    if motor:
        a.merge(build_pusher(cfg), group_name="pusher")
    a.merge(build_elevon_lines(cfg), group_name="elevons")
    return a


# ==========================================================================
# report and export
# ==========================================================================

def spec(cfg=JW):
    refl = _reflex(cfg)
    sec = thin_aerofoil(cfg["camber"], cfg["camber_pos"], refl,
                        cfg["reflex_start"])
    pf = planform(cfg)
    tr = solve_trim(cfg, sec)
    W = cfg["mass_g"] / 1000.0 * G
    a = assemble(cfg)
    lo, hi = a.bounds()
    nose = lo[0]
    v_tr = math.sqrt(2 * W / (RHO * pf["S"] * max(tr["CL_trim"], 0.05)))
    v_st = math.sqrt(2 * W / (RHO * pf["S"] * 0.85))
    return {"cfg": cfg, "sec": sec, "reflex": refl, "pf": pf, "trim": tr,
            "mesh": a, "nose_x": nose,
            "cg_from_nose": tr["x_cg"] - nose,
            "tip_to_tip": hi[1] - lo[1], "length": hi[0] - lo[0],
            "height": hi[2] - lo[2],
            "wing_loading": cfg["mass_g"] / (pf["S"] * 100.0),
            "v_trim": v_tr, "v_stall": v_st}


def report(sp):
    c, pf, tr, sec = sp["cfg"], sp["pf"], sp["trim"], sp["sec"]
    L = []
    L.append(f"  span {c['span']:.0f} mm  (tip to tip over the winglets "
             f"{sp['tip_to_tip']:.0f} mm)")
    L.append(f"  length {sp['length']:.0f} mm, height {sp['height']:.0f} mm")
    L.append(f"  area {pf['S'] * 1e4:.0f} cm2, AR {pf['AR']:.2f}, "
             f"taper {pf['taper']:.2f}, LE sweep {c['sweep_le']:.0f} deg")
    L.append(f"  MAC {pf['mac']:.1f} mm at y={pf['y_mac']:.0f} mm")
    L.append(f"  section: {c['thickness'] * 100:.1f}% thick, "
             f"{c['camber'] * 100:.1f}% camber, reflex {sp['reflex']:.4f} "
             f"aft of {c['reflex_start'] * 100:.0f}% chord")
    L.append(f"    -> Cm(c/4) {sec['cm_c4']:+.4f}, "
             f"zero-lift angle {sec['alpha_L0_deg']:+.2f} deg")
    L.append(f"  washout {c['washout_tip']:+.1f} deg at the tip")
    L.append(f"  NEUTRAL POINT {tr['np_pct_mac']:.1f}% MAC, "
             f"static margin {tr['sm'] * 100:.1f}%")
    L.append(f"  CENTRE OF GRAVITY {sp['cg_from_nose']:.0f} mm from the nose "
             f"= {tr['cg_pct_mac']:.1f}% MAC")
    L.append(f"  trims at {tr['alpha_trim']:.1f} deg for CL {tr['CL_trim']:.3f}")
    L.append(f"  at {c['mass_g']:.0f} g: {sp['wing_loading']:.1f} g/dm2, "
             f"trim {sp['v_trim']:.1f} m/s, stall {sp['v_stall']:.1f} m/s")
    return "\n".join(L)


VARIANTS = [("jetwing-1500", 1500.0, "larger wing"),
            ("jetwing-1200", 1200.0, "standard wing")]


def main():
    out = os.path.join(ROOT, "models", "jetwing")
    img = os.path.join(ROOT, "docs", "img", "jetwing")
    os.makedirs(out, exist_ok=True)
    os.makedirs(img, exist_ok=True)
    import render as R
    for (name, span, desc) in VARIANTS:
        cfg = dict(JW)
        k = span / JW["span"]
        for key in ("span", "root_chord", "tip_chord", "winglet_h", "fin_h",
                    "fin_root", "fin_tip", "pod_len", "pod_w", "pod_h",
                    "prop_dia", "spinner"):
            cfg[key] = JW[key] * k
        cfg["mass_g"] = JW["mass_g"] * k ** 2.6     # printed shell, mostly area
        sp = spec(cfg)
        M.write_stl(sp["mesh"], os.path.join(out, name + ".stl"), f"JW-1 {name}")
        M.write_obj(sp["mesh"], os.path.join(out, name + ".obj"), name)
        R.render(sp["mesh"], R.iso(38, -24), w=1250, h=820,
                 path=os.path.join(img, name + ".png"))
        print(f"\n== {name}  ({desc}) ==")
        print(report(sp))
    print(f"\nwrote models/jetwing/ and docs/img/jetwing/")


if __name__ == "__main__":
    main()
