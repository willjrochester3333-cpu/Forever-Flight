"""
Airfoil section geometry for Forever-Flight.

Two things live here:

1. An exact NACA 4-digit generator (closed-form, verifiable) used as the
   geometric base for the mould definition.

2. A "solar flattening" morph. Encapsulated monocrystalline cells crack if you
   bend them too hard. The morph relaxes upper-surface curvature inside a
   chordwise band until the local radius everywhere exceeds a limit, so the
   cell strips can be vacuum-bagged onto the skin without micro-cracking.

The resulting section is called FF-SC1 (Forever-Flight, Solar Chord, rev 1).

IMPORTANT / honesty note
------------------------
FF-SC1 is a *geometric mould definition* for the 3D model and the plug. It is
not a validated low-Reynolds aerofoil. The aerodynamic specification for the
flight article is SD7037 (or AG40d-series) with the same flattening morph
applied, verified in XFOIL/XFLR5 at Re = 90k-160k with an N_crit of 9 before
any mould is cut. Drop a Selig-format .dat into airfoils/ and point
config.py at it to regenerate the model on the real section.
"""

from __future__ import annotations

import math


# --------------------------------------------------------------------------
# NACA 4-digit
# --------------------------------------------------------------------------

def _thickness(x, t, blunt_te=True):
    """Half-thickness of the NACA 4-digit distribution at x/c."""
    a4 = -0.1015 if not blunt_te else -0.1036  # -0.1036 closes the TE exactly
    return 5.0 * t * (0.2969 * math.sqrt(max(x, 0.0))
                      - 0.1260 * x
                      - 0.3516 * x * x
                      + 0.2843 * x ** 3
                      + a4 * x ** 4)


def _camber(x, m, p):
    if m == 0.0:
        return 0.0, 0.0
    if x < p:
        yc = (m / (p * p)) * (2 * p * x - x * x)
        dy = (2 * m / (p * p)) * (p - x)
    else:
        q = 1.0 - p
        yc = (m / (q * q)) * ((1 - 2 * p) + 2 * p * x - x * x)
        dy = (2 * m / (q * q)) * (p - x)
    return yc, dy


def naca4(m, p, t, n_per_side=90):
    """Return (upper, lower) lists of (x, y) in chord fractions, LE -> TE.

    m = max camber (fraction of chord), p = its chordwise position,
    t = max thickness (fraction of chord).
    """
    upper, lower = [], []
    for i in range(n_per_side + 1):
        beta = math.pi * i / n_per_side
        x = 0.5 * (1.0 - math.cos(beta))          # cosine clustering at LE/TE
        yt = _thickness(x, t)
        yc, dy = _camber(x, m, p)
        th = math.atan(dy)
        upper.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lower.append((x + yt * math.sin(th), yc - yt * math.cos(th)))
    return upper, lower


# --------------------------------------------------------------------------
# curvature tools
# --------------------------------------------------------------------------

def curvature_radii(pts):
    """Circumradius through each interior triple. Large R = flat."""
    out = []
    for i in range(1, len(pts) - 1):
        (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[i + 1]
        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        area2 = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
        out.append(float("inf") if area2 < 1e-14 else (a * b * c) / (2.0 * area2))
    return out


def min_radius_in_band(pts, x0, x1):
    radii = curvature_radii(pts)
    best = float("inf")
    for i, r in enumerate(radii, start=1):
        if x0 <= pts[i][0] <= x1:
            best = min(best, r)
    return best


def solar_flatten(upper, x0, x1, chord_mm, r_min_mm, max_iter=6000):
    """Relax upper-surface curvature inside [x0, x1] (chord fractions).

    Endpoints of the band are clamped, so the section stays continuous and the
    leading edge and trailing edge are untouched. Laplacian smoothing only ever
    reduces curvature, so this converges monotonically.
    """
    pts = [list(p) for p in upper]
    idx = [i for i, p in enumerate(pts) if x0 < p[0] < x1]
    if not idx:
        return [tuple(p) for p in pts], float("inf"), 0

    r_min_c = r_min_mm / chord_mm  # target radius in chord fractions
    it = 0
    for it in range(1, max_iter + 1):
        cur = min_radius_in_band([(p[0], p[1]) for p in pts], x0, x1)
        if cur >= r_min_c:
            break
        for i in idx:
            xm, xp = pts[i - 1][0], pts[i + 1][0]
            ym, yp = pts[i - 1][1], pts[i + 1][1]
            # linear interpolation onto the chord of the neighbours
            w = (pts[i][0] - xm) / (xp - xm) if xp > xm else 0.5
            target = ym + w * (yp - ym)
            pts[i][1] += 0.18 * (target - pts[i][1])

    final = min_radius_in_band([(p[0], p[1]) for p in pts], x0, x1)
    return [tuple(p) for p in pts], final * chord_mm, it


# --------------------------------------------------------------------------
# section assembly
# --------------------------------------------------------------------------

def closed_loop(upper, lower, te_gap=0.0):
    """TE-upper -> LE -> TE-lower, as one closed polygon.

    te_gap opens a blunt trailing edge (fraction of chord) so the moulded skin
    has a real, printable/bondable thickness at the TE.
    """
    up = list(upper)
    lo = list(lower)
    if te_gap > 0.0:
        for seq, sign in ((up, +1.0), (lo, -1.0)):
            for i, (x, y) in enumerate(seq):
                seq[i] = (x, y + sign * 0.5 * te_gap * x ** 2)
    loop = [p for p in reversed(up)]        # TE -> LE along the top
    loop += [p for p in lo[1:]]             # LE -> TE along the bottom
    return loop


def section_properties(upper, lower):
    n = min(len(upper), len(lower))
    t_max, t_at = 0.0, 0.0
    c_max, c_at = 0.0, 0.0
    for i in range(n):
        xu, yu = upper[i]
        xl, yl = lower[i]
        t = yu - yl
        cm = 0.5 * (yu + yl)
        if t > t_max:
            t_max, t_at = t, 0.5 * (xu + xl)
        if abs(cm) > abs(c_max):
            c_max, c_at = cm, 0.5 * (xu + xl)
    area = 0.0
    for i in range(n - 1):
        area += 0.5 * ((upper[i][1] - lower[i][1]) + (upper[i + 1][1] - lower[i + 1][1])) \
                * (upper[i + 1][0] - upper[i][0])
    return {"t_max": t_max, "t_at": t_at,
            "camber_max": c_max, "camber_at": c_at, "area": area}


def load_selig_dat(path):
    """Read a UIUC/Selig .dat file -> (upper, lower), both LE -> TE."""
    xs = []
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) != 2:
                continue
            try:
                xs.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    if not xs:
        raise ValueError(f"no coordinates found in {path}")
    split = min(range(len(xs)), key=lambda i: xs[i][0])
    upper = list(reversed(xs[:split + 1]))
    lower = xs[split:]
    return upper, lower


def ff_sc1(chord_mm, t=0.095, m=0.028, p=0.42,
           band=(0.18, 0.90), r_min_mm=520.0, te_gap=0.004, n_per_side=90):
    """The Forever-Flight solar-chord section, at a given chord."""
    upper, lower = naca4(m, p, t, n_per_side=n_per_side)
    flat, r_final, iters = solar_flatten(upper, band[0], band[1], chord_mm, r_min_mm)
    props = section_properties(flat, lower)
    props.update({"r_min_mm": r_final, "iters": iters, "chord_mm": chord_mm,
                  "band": band, "r_target_mm": r_min_mm})
    return closed_loop(flat, lower, te_gap=te_gap), props
