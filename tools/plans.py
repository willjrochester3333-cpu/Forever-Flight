#!/usr/bin/env python3
"""
Printable 1:1 build plans for the FF-1 airframe, tiled onto A4.

    python3 tools/plans.py            -> plans/FF-1_plans_A4.pdf + plans/dxf/

What this gives you: the AERODYNAMIC SHAPE of the FF-1 as a foam prototype --
wing planform, aerofoil templates, fuselage pod, V-tail -- at exact full size,
spread over A4 sheets you tape together. It is phase 1 of the flight test plan
in docs/DESIGN.md section 11, and it is the right first build: cheap, quick,
and it produces the measured sink polar that everything else depends on.

What it does NOT give you: the solar array, the energy-recovery turbine or the
retractable motor pylon. Those need the moulded composite airframe -- a foam
prototype has neither the stiffness for a flat cell array nor the internal
volume for the mast trunk. Build this first, fly it, log it, then commit to
moulds.

PRINTING: 100 % / "actual size". Never "fit to page" or "shrink to fit",
which silently take 4-6 % off and put the CG in the wrong place. Every sheet
carries a 100 mm bar -- measure it before cutting anything.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analysis as AN
import build_model as B
import config as C
import pdf as P

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "plans")

MARGIN = 9.0          # mm, unprintable edge allowance
OVERLAP = 12.0        # mm, glue flap between tiles
LW_CUT = 0.45         # mm line width, cut lines
LW_MARK = 0.22        # reference lines
LW_FINE = 0.13        # grid, hatching

# Foam prototype mass budget (no solar, no turbine, no mast) -- grams
PROTO_MASS = [
    ("Wing cores, XPS 30 kg/m3", 112.0),
    ("Wing skin: 25 g/m2 glass or laminating film", 78.0),
    ("Wing spar, 8 mm carbon tube + joiner", 52.0),
    ("Control surfaces, hinges, horns", 22.0),
    ("Fuselage pod, 3 mm depron + ply doublers", 86.0),
    ("Tail boom, 16-12 mm carbon, 700 mm", 31.0),
    ("V-tail panels + mount", 44.0),
    ("Motor, ESC, folding prop (nose)", 92.0),
    ("Battery, 3S 2200 mAh LiPo", 180.0),
    ("RX, 4 servos, wiring", 68.0),
    ("Flight controller, GNSS, airspeed", 52.0),
]
PROTO_CD0 = 0.0310    # foam build: rougher surface, blunter LE, exposed linkages


# ==========================================================================
# drawing model -- everything in millimetres
# ==========================================================================

class Drawing:
    """A flat list of primitives with bounding boxes, so tiles can reject."""

    def __init__(self, name):
        self.name = name
        self.items = []

    def _bb(self, pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))

    def poly(self, pts, kind="cut", close=False, layer=None):
        if len(pts) > 1:
            self.items.append({"t": "poly", "pts": list(pts), "kind": kind,
                               "close": close, "bb": self._bb(pts),
                               "layer": layer or kind})
        return self

    def line(self, x0, y0, x1, y1, kind="mark", layer=None):
        return self.poly([(x0, y0), (x1, y1)], kind=kind, layer=layer)

    def circle(self, cx, cy, r, kind="mark"):
        self.items.append({"t": "circ", "c": (cx, cy), "r": r, "kind": kind,
                           "bb": (cx - r, cy - r, cx + r, cy + r),
                           "layer": kind})
        return self

    def text(self, x, y, s, size=3.2, bold=False, rot=0.0, align="l",
             kind="text"):
        w = P.Page.text_width(s, size, bold)
        # Rotated text runs along its own axis; a square bbox would inflate the
        # drawing extent and force extra tile columns.
        if abs(abs(rot) - 90.0) < 45.0:
            bb = (x - size, y - size, x + size, y + w + size)
        else:
            dx0 = {"l": 0.0, "c": w / 2.0, "r": w}[align]
            bb = (x - dx0, y - size, x - dx0 + w, y + size)
        self.items.append({"t": "text", "p": (x, y), "s": s, "size": size,
                           "bold": bold, "rot": rot, "align": align,
                           "kind": kind, "layer": kind, "bb": bb})
        return self

    def dim(self, x0, y0, x1, y1, label, off=0.0):
        """Simple dimension line with arrow ticks and a centred label."""
        self.poly([(x0, y0), (x1, y1)], kind="dim")
        a = math.atan2(y1 - y0, x1 - x0)
        for (px, py) in ((x0, y0), (x1, y1)):
            dx, dy = 2.0 * math.cos(a + 2.4), 2.0 * math.sin(a + 2.4)
            self.poly([(px, py), (px + dx, py + dy)], kind="dim")
            dx, dy = 2.0 * math.cos(a - 2.4), 2.0 * math.sin(a - 2.4)
            self.poly([(px, py), (px + dx, py + dy)], kind="dim")
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        deg = math.degrees(a)
        if deg > 90 or deg < -90:
            deg += 180
        self.text(mx, my + off + 1.2, label, size=3.0, rot=deg, align="c",
                  kind="dim")
        return self

    def bbox(self):
        if not self.items:
            return (0, 0, 1, 1)
        x0 = min(i["bb"][0] for i in self.items)
        y0 = min(i["bb"][1] for i in self.items)
        x1 = max(i["bb"][2] for i in self.items)
        y1 = max(i["bb"][3] for i in self.items)
        return (x0, y0, x1, y1)


STYLE = {
    "cut":   (LW_CUT, 0.0, None),
    "fold":  (LW_MARK, 0.35, [4, 2]),
    "mark":  (LW_MARK, 0.45, [3, 2]),
    "spar":  (LW_MARK, 0.15, [8, 2, 2, 2]),
    "hinge": (LW_MARK, 0.2, [6, 2]),
    "cell":  (LW_FINE, 0.6, None),
    "dim":   (LW_FINE, 0.4, None),
    "text":  (LW_FINE, 0.0, None),
    "guide": (LW_FINE, 0.72, [1.5, 1.5]),
}


def render(page, dw, clip=None):
    """Draw a Drawing onto a page whose frame is already in mm."""
    for it in dw.items:
        if clip:
            cx0, cy0, cx1, cy1 = clip
            bx0, by0, bx1, by1 = it["bb"]
            if bx1 < cx0 - 2 or bx0 > cx1 + 2 or by1 < cy0 - 2 or by0 > cy1 + 2:
                continue
        lw, grey, dash = STYLE.get(it["kind"], STYLE["mark"])
        page.width(lw).grey(grey).dash(dash)
        if it["t"] == "poly":
            page.poly(it["pts"], close=it["close"])
        elif it["t"] == "circ":
            page.circle(it["c"][0], it["c"][1], it["r"])
        elif it["t"] == "text":
            page.grey(grey, fill=True)
            page.text(it["p"][0], it["p"][1], it["s"], size=it["size"],
                      bold=it["bold"], rot=it["rot"], align=it["align"])


# ==========================================================================
# page furniture
# ==========================================================================

def scale_bar(page, x, y, label="CHECK BEFORE CUTTING"):
    """100 mm bar with 10 mm ticks. If this is not 100 mm, the print is wrong."""
    page.save()
    page.width(0.3).grey(0.0).dash(None)
    page.line(x, y, x + 100, y)
    for i in range(11):
        h = 3.0 if i % 5 == 0 else 1.8
        page.line(x + i * 10, y, x + i * 10, y + h)
    page.rect(x, y, 10, 2.6, stroke=False, fill=True)
    page.rect(x + 20, y, 10, 2.6, stroke=False, fill=True)
    page.rect(x + 40, y, 10, 2.6, stroke=False, fill=True)
    page.grey(0.0, fill=True)
    page.text(x, y + 4.6, "0", size=2.6)
    page.text(x + 50, y + 4.6, "50 mm", size=2.6, align="c")
    page.text(x + 100, y + 4.6, "100 mm", size=2.6, align="r")
    page.text(x, y - 3.4, label, size=2.5, bold=True)
    page.restore()


def frame(page, title, sub, sheet, total, landscape=True):
    """Title block along the bottom edge, plus the scale bar."""
    w = page.w
    page.save().mm_frame()
    page.width(0.25).grey(0.55).dash(None)
    page.line(MARGIN, MARGIN + 13.0, w - MARGIN, MARGIN + 13.0)
    page.grey(0.0, fill=True)
    page.text(MARGIN, MARGIN + 8.0, title, size=4.0, bold=True)
    page.grey(0.35, fill=True)
    page.text(MARGIN, MARGIN + 3.2, sub, size=2.8)
    page.grey(0.0, fill=True)
    page.text(w - MARGIN, MARGIN + 8.0, f"SHEET {sheet} / {total}",
              size=3.4, bold=True, align="r")
    page.grey(0.35, fill=True)
    page.text(w - MARGIN, MARGIN + 3.2, "FF-1 Ember  -  print at 100%",
              size=2.6, align="r")
    scale_bar(page, w / 2.0 - 50.0, MARGIN + 3.0)
    page.restore()


def reg_marks(page, x0, y0, x1, y1, col, row, ncols, nrows, ov):
    """Overlap bands and alignment marks so tiles can be taped accurately."""
    page.save().mm_frame()
    page.width(0.2).grey(0.75).dash([1.5, 1.5])
    if col < ncols - 1:
        page.line(x1 - ov, y0, x1 - ov, y1)
    if row < nrows - 1:
        page.line(x0, y1 - ov, x1, y1 - ov)
    page.dash(None).grey(0.0).width(0.35)
    for (mx, my) in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
        page.line(mx - 3, my, mx + 3, my)
        page.line(mx, my - 3, mx, my + 3)
    # edge triangles: match these across the seam
    page.grey(0.0, fill=True)
    for frac in (0.25, 0.5, 0.75):
        if col < ncols - 1:
            yy = y0 + (y1 - y0) * frac
            page.poly([(x1 - ov, yy - 2), (x1 - ov + 3.2, yy),
                       (x1 - ov, yy + 2)], close=True, stroke=False, fill=True)
        if row < nrows - 1:
            xx = x0 + (x1 - x0) * frac
            page.poly([(xx - 2, y1 - ov), (xx, y1 - ov + 3.2),
                       (xx + 2, y1 - ov)], close=True, stroke=False, fill=True)
    page.grey(0.45, fill=True)
    page.text(x0 + 2, y1 - 4, f"R{row + 1}C{col + 1}", size=3.0, bold=True)
    page.restore()


def tile(doc, dw, title, pages_so_far, total_est, landscape=True, pad=6.0):
    """Lay a Drawing across as many A4 sheets as it needs."""
    pw, ph = (P.A4[1], P.A4[0]) if landscape else P.A4
    uw = pw - 2 * MARGIN
    uh = ph - 2 * MARGIN - 15.0          # room for the title block
    x0, y0, x1, y1 = dw.bbox()
    x0 -= pad
    y0 -= pad
    x1 += pad
    y1 += pad
    W, H = x1 - x0, y1 - y0
    step_w, step_h = uw - OVERLAP, uh - OVERLAP
    ncols = max(1, math.ceil((W - OVERLAP) / step_w))
    nrows = max(1, math.ceil((H - OVERLAP) / step_h))
    made = []
    for row in range(nrows):
        for col in range(ncols):
            tx0 = x0 + col * step_w
            ty0 = y0 + row * step_h
            tx1, ty1 = tx0 + uw, ty0 + uh
            pg = doc.page(pw, ph)
            pg.save().mm_frame(1.0, MARGIN - tx0, MARGIN + 15.0 - ty0)
            pg.save().clip_rect(tx0, ty0, uw, uh)
            render(pg, dw, clip=(tx0, ty0, tx1, ty1))
            pg.restore()
            pg.restore()
            reg_marks(pg, MARGIN, MARGIN + 15.0, MARGIN + uw, MARGIN + 15.0 + uh,
                      col, row, ncols, nrows, OVERLAP)
            frame(pg, title, f"tile row {row + 1} of {nrows}, column "
                             f"{col + 1} of {ncols}  -  overlap "
                             f"{OVERLAP:.0f} mm, align the triangles",
                  pages_so_far + len(made) + 1, total_est)
            made.append(pg)
    return made


# ==========================================================================
# parts
# ==========================================================================

def _wing_stations():
    """(y, chord) with the leading edge straight at x = 0."""
    return [(s["y"], s["chord"]) for s in B.wing_geometry()]


def chord_at(y):
    st = _wing_stations()
    for i in range(len(st) - 1):
        if st[i][0] <= y <= st[i + 1][0]:
            t = (y - st[i][0]) / (st[i + 1][0] - st[i][0])
            return st[i][1] + t * (st[i + 1][1] - st[i][1])
    return st[-1][1]


def wing_planform(show_cells=True):
    dw = Drawing("wing")
    st = _wing_stations()
    tip_y = st[-1][0]

    te = [(c, y) for (y, c) in st]
    outline = [(0.0, 0.0), (0.0, tip_y)] + [(st[-1][1], tip_y)] + \
        list(reversed(te[:-1])) + [(st[0][1], 0.0)]
    dw.poly(outline, kind="cut", close=True)

    # spar, hinge lines, CG
    n = 40
    spar = [(0.30 * chord_at(tip_y * i / n), tip_y * i / n) for i in range(n + 1)]
    dw.poly(spar, kind="spar")
    dw.text(0.30 * chord_at(tip_y * 0.55) + 3, tip_y * 0.55,
            "MAIN SPAR  30% chord", size=3.0, rot=90)

    for key, label in (("aileron", "AILERON"), ("flap", "FLAP")):
        s = C.WING[key]
        cf = 1.0 - s["chord_frac"]
        pts = []
        yy = s["y0"]
        while yy <= s["y1"] + 1e-9:
            pts.append((cf * chord_at(yy), yy))
            yy += 20.0
        dw.poly(pts, kind="hinge")
        dw.poly([(cf * chord_at(s["y0"]), s["y0"]),
                 (chord_at(s["y0"]), s["y0"])], kind="hinge")
        dw.poly([(cf * chord_at(s["y1"]), s["y1"]),
                 (chord_at(s["y1"]), s["y1"])], kind="hinge")
        dw.text(cf * chord_at((s["y0"] + s["y1"]) / 2) + 4,
                (s["y0"] + s["y1"]) / 2, f"{label}  {s['chord_frac'] * 100:.0f}% chord",
                size=3.0, rot=90)

    # dihedral / panel break
    yb = C.WING["stations"][1][0]
    dw.poly([(0, yb), (chord_at(yb), yb)], kind="fold")
    dw.text(chord_at(yb) + 4, yb, f"PANEL BREAK  y={yb:.0f}   "
            f"outer panel up {C.WING['stations'][2][3]:.1f} deg dihedral",
            size=3.0, rot=90)

    # centre of gravity -- the number that decides whether it flies
    st_ = AN.stability()
    g = AN.geom()
    cg_x = st_["cg_target"] * g["mac"] * 1000.0
    dw.poly([(cg_x, -14), (cg_x, 130)], kind="guide")
    dw.circle(cg_x, 60, 4.0, kind="cut")
    dw.circle(cg_x, 60, 1.4, kind="cut")
    dw.text(cg_x + 7, 52, f"C of G  {cg_x:.0f} mm aft of LE", size=3.6, bold=True)
    dw.text(cg_x + 7, 46, "balance here, wing level, nose slightly down",
            size=2.8)
    dw.dim(0, -8, cg_x, -8, f"{cg_x:.0f}")

    # ribs / hot-wire template stations
    for (y, c) in st:
        dw.poly([(0, y), (c, y)], kind="mark")
        dw.text(-4, y, f"y={y:.0f}  c={c:.0f}", size=2.8, align="r")

    if show_cells:
        for s in B.solar_layout()["strips"]:
            c = s["chord"]
            dw.poly([(s["xc0"] * c, s["y0"]), (s["xc1"] * c, s["y0"]),
                     (s["xc1"] * c, s["y1"]), (s["xc0"] * c, s["y1"])],
                    kind="cell", close=True)
        dw.text(6, tip_y - 120, "faint boxes: solar cell strips on the",
                size=2.7, kind="dim")
        dw.text(6, tip_y - 125, "composite version -- ignore for foam",
                size=2.7, kind="dim")

    dw.dim(-22, 0, -22, tip_y, f"SEMI-SPAN {tip_y:.0f}")
    dw.dim(0, tip_y + 16, st[-1][1], tip_y + 16, f"TIP {st[-1][1]:.0f}")
    dw.text(12, 20, "WING - CUT 2, ONE MIRRORED", size=5.0, bold=True)
    dw.text(12, 13, f"washout: twist tip {abs(C.WING['stations'][-1][4]):.1f} deg "
            f"trailing edge UP, progressive from the panel break", size=3.0)
    dw.text(12, 7, f"section FF-SC1 {C.WING['airfoil']['thickness'] * 100:.1f}% "
            f"- see the aerofoil template sheets", size=3.0)
    return dw


def airfoil_template(chord, label, spar_tube=8.0):
    dw = Drawing(f"af{chord:.0f}")
    _loop, upper, _props = B.wing_section()
    lo = [(p[0], p[1]) for p in _loop]
    pts = [(x * chord, y * chord) for (x, y) in lo]
    dw.poly(pts, kind="cut", close=True)
    dw.poly([(0, 0), (chord, 0)], kind="mark")
    for f in (0.0, 0.25, 0.30, 0.5, 0.75, 1.0):
        dw.poly([(f * chord, -3), (f * chord, 3)], kind="dim")
        dw.text(f * chord, -7, f"{f * 100:.0f}%", size=2.6, align="c")
    # Centre the spar on the section's MID-THICKNESS at 30 % chord, which
    # needs both surfaces -- taking half the upper ordinate puts it too high.
    f = C.WING["spar_at_chord"]
    ups = [(x, y) for (x, y) in lo[:len(lo) // 2 + 1]]
    los = [(x, y) for (x, y) in lo[len(lo) // 2:]]

    def surf_y(seq, xf):
        seq = sorted(seq, key=lambda q: q[0])
        for i in range(len(seq) - 1):
            if seq[i][0] <= xf <= seq[i + 1][0]:
                t = ((xf - seq[i][0]) / (seq[i + 1][0] - seq[i][0])
                     if seq[i + 1][0] > seq[i][0] else 0.0)
                return seq[i][1] + t * (seq[i + 1][1] - seq[i][1])
        return seq[-1][1]

    yu, yl = surf_y(ups, f), surf_y(los, f)
    thick = abs(yu - yl) * chord
    sx = f * chord
    yc = 0.5 * (yu + yl) * chord
    tube = min(spar_tube, thick - 2.0)
    dw.circle(sx, yc, tube / 2.0, kind="mark")
    top = max(y for (_x, y) in pts)
    dw.poly([(sx, yc + tube / 2.0), (sx, top + 3.0)], kind="dim")
    dw.text(sx, top + 4.2, f"spar {tube:.0f} mm  (section {thick:.1f} mm here)",
            size=2.5, align="c")
    dw.text(0, chord * 0.14, label, size=4.2, bold=True)
    dw.text(0, chord * 0.14 - 5.5, f"chord {chord:.0f} mm", size=3.0)
    dw.dim(0, -13, chord, -13, f"{chord:.0f}")
    return dw


def fuselage_pod(x_max=620.0):
    dw = Drawing("pod")
    st = [s for s in C.FUSELAGE["stations"] if s[0] <= x_max + 1e-9]
    top = [(x, cz + h / 2) for (x, w, h, cz) in st]
    bot = [(x, cz - h / 2) for (x, w, h, cz) in st]
    dw.poly(top + list(reversed(bot)), kind="cut", close=True)
    dw.poly([(0, 0), (x_max, 0)], kind="mark")

    t = C.FUSELAGE["turret"]
    dw.circle(t["x"], t["z"], t["r"], kind="fold")
    dw.text(t["x"] + t["r"] + 3, t["z"], "sensor bay (optional on the prototype)",
            size=2.7)

    wle = C.WING["stations"][0][2]
    wc = C.WING["stations"][0][1]
    wz = C.WING["root_z"]
    dw.poly([(wle, wz), (wle + wc, wz)], kind="hinge")
    dw.text(wle + 4, wz + 3, f"WING SEAT  LE {wle:.0f}, TE {wle + wc:.0f}, "
            f"incidence +{C.WING['incidence']:.1f} deg", size=2.9)

    st_ = AN.stability()
    g = AN.geom()
    cg = wle + st_["cg_target"] * g["mac"] * 1000.0
    dw.poly([(cg, -56), (cg, 56)], kind="guide")
    dw.circle(cg, 0, 4.0, kind="cut")
    dw.text(cg + 6, -30, f"C of G  {cg:.0f} mm from nose", size=3.6, bold=True)

    for name, (a, b) in sorted(C.FUSELAGE["bays"].items(), key=lambda kv: kv[1][0]):
        if a >= x_max:
            continue
        b = min(b, x_max)
        dw.poly([(a, -46), (a, -52), (b, -52), (b, -46)], kind="dim")
        dw.text((a + b) / 2, -56, name.replace("_", " "), size=2.6, align="c")

    # plan view, below
    dy = -112.0
    ph = [(x, dy + w / 2) for (x, w, h, cz) in st]
    pl = [(x, dy - w / 2) for (x, w, h, cz) in st]
    dw.poly(ph + list(reversed(pl)), kind="cut", close=True)
    dw.poly([(0, dy), (x_max, dy)], kind="mark")
    dw.text(6, dy + 34, "PLAN VIEW", size=4.0, bold=True)
    dw.text(6, 44, "SIDE VIEW - cut 2 sides from 3 mm depron", size=4.0, bold=True)
    dw.text(6, 38, f"boom socket at x={x_max:.0f}: 16 mm carbon tube, "
            f"{C.VTAIL['le_x'] - x_max + 90:.0f} mm long", size=3.0)
    dw.dim(0, 62, x_max, 62, f"POD LENGTH {x_max:.0f}")
    return dw


def formers(x_max=620.0):
    dw = Drawing("formers")
    st = [s for s in C.FUSELAGE["stations"] if 20.0 <= s[0] <= x_max]
    n = C.FUSELAGE["superellipse_n"]
    x = 0.0
    row_h = 0.0
    for (sx, w, h, cz) in st:
        ring = B.M.superellipse(w, h, n=n, npts=48, cz=0.0)
        pts = [(x + p[1], p[2]) for p in ring]
        dw.poly(pts, kind="cut", close=True)
        dw.poly([(x - w / 2 - 2, 0), (x + w / 2 + 2, 0)], kind="dim")
        dw.poly([(x, -h / 2 - 2), (x, h / 2 + 2)], kind="dim")
        dw.text(x, -h / 2 - 7, f"F{sx:.0f}", size=3.0, align="c", bold=True)
        dw.text(x, -h / 2 - 11, f"{w:.0f}x{h:.0f}", size=2.5, align="c")
        x += w + 16.0
        row_h = max(row_h, h)
    dw.text(0, row_h / 2 + 12, "FORMERS - cut from 3 mm ply or 5 mm depron",
            size=4.0, bold=True)
    dw.text(0, row_h / 2 + 6, "F<number> is the station in mm from the nose; "
            "the horizontal tick is the fuselage datum", size=2.9)
    return dw


def vtail_panel():
    dw = Drawing("vtail")
    v = C.VTAIL
    sw = math.tan(math.radians(v["sweep_le"]))
    span, rc, tc = v["panel_span"], v["root_chord"], v["tip_chord"]
    dw.poly([(0, 0), (span * sw, span), (span * sw + tc, span), (rc, 0)],
            kind="cut", close=True)
    cf = 1.0 - v["ruddervator_chord_frac"]
    pts = []
    yy = 0.0
    while yy <= span:
        c = rc + (tc - rc) * yy / span
        pts.append((yy * sw + cf * c, yy))
        yy += 20.0
    dw.poly(pts, kind="hinge")
    dw.text(cf * rc + 4, span * 0.45,
            f"RUDDERVATOR  {v['ruddervator_chord_frac'] * 100:.0f}% chord",
            size=3.0, rot=90)
    dw.dim(-14, 0, -14, span, f"PANEL SPAN {span:.0f}")
    dw.dim(0, -12, rc, -12, f"ROOT {rc:.0f}")
    dw.text(8, span - 30, "V-TAIL - CUT 2, ONE MIRRORED", size=4.6, bold=True)
    dw.text(8, span - 37, f"dihedral {v['dihedral']:.0f} deg from horizontal "
            f"({2 * v['dihedral']:.0f} deg included)", size=3.0)
    dw.text(8, span - 43, f"incidence {v['incidence']:.1f} deg "
            f"(decalage {C.WING['incidence'] - v['incidence']:.1f} deg "
            f"nose-up on the wing)", size=3.0)
    dw.text(8, span - 49, "symmetric section, 9% thick - or flat 5 mm foam "
            "with a rounded LE", size=3.0)
    return dw


# ==========================================================================
# prototype performance
# ==========================================================================

def proto_performance():
    """What the foam prototype should do, on its own mass and drag."""
    mass = sum(g for _n, g in PROTO_MASS) / 1000.0
    W = mass * C.MISSION["g"]
    g = AN.geom()
    saved = (C.DRAG["components"], C.DRAG["margin"])
    try:
        C.DRAG["components"] = [("foam prototype", PROTO_CD0 - C.DRAG["wing_cd_min"], "")]
        C.DRAG["margin"] = 0.0
        kp = AN.key_points(W)
        pms = AN.practical_min_sink(W)
        return {"mass_kg": mass, "W": W, "wing_loading": W / g["S"],
                "cd0": PROTO_CD0, "ld": kp["best_glide"]["LD"],
                "v_bg": kp["best_glide"]["V"], "sink_bg": kp["best_glide"]["sink"],
                "v_stall": kp["v_stall"], "sink_min": pms["sink"],
                "v_ms": pms["V"]}
    finally:
        C.DRAG["components"], C.DRAG["margin"] = saved


# ==========================================================================
# front matter
# ==========================================================================

def page_title(doc, sheets):
    pg = doc.page(P.A4[0], P.A4[1])
    pg.save().mm_frame()
    W, H = P.A4
    y = H - 24

    pg.grey(0.0, fill=True)
    pg.text(MARGIN, y, "FF-1 EMBER", size=11.0, bold=True)
    pg.text(MARGIN, y - 8, "Full-size build plans, tiled for A4", size=5.0)
    y -= 20

    # print-check block -- the most important thing on the sheet
    pg.width(0.7).grey(0.0).dash(None)
    pg.rect(MARGIN, y - 42, W - 2 * MARGIN, 42)
    pg.grey(0.0, fill=True)
    pg.text(MARGIN + 5, y - 9, "PRINT AT 100 %  -  ACTUAL SIZE", size=7.0, bold=True)
    pg.text(MARGIN + 5, y - 15.5,
            "In the print dialog set Scale to 100 %. Turn OFF \"Fit to page\", "
            "\"Shrink oversized pages\" and \"Scale to fit\".", size=3.2)
    pg.text(MARGIN + 5, y - 20.5,
            "Those options take 4-6 % off, which moves the centre of gravity "
            "and will make the aircraft unflyable.", size=3.2)
    scale_bar(pg, MARGIN + 5, y - 33, "MEASURE THIS BAR - IT MUST BE 100 mm")
    pg.width(0.35).grey(0.0).dash(None)
    pg.rect(W - MARGIN - 38, y - 34, 25, 25)
    pg.grey(0.0, fill=True)
    pg.text(W - MARGIN - 38, y - 37.5, "25 mm square", size=2.6)
    y -= 50

    pg.grey(0.0, fill=True)
    pg.text(MARGIN, y, "WHAT THIS IS", size=5.0, bold=True)
    y -= 6
    for ln in ("A foam prototype of the FF-1's aerodynamic shape: wing, tail and",
               "fuselage pod at exact full size. It is phase 1 of the flight test",
               "plan - the aircraft that gets you a measured sink polar before you",
               "commit to composite moulds.",
               "",
               "It is NOT the solar aircraft. The array, the energy-recovery",
               "turbine and the retractable motor pylon all need the moulded",
               "airframe: foam has neither the stiffness for a flat cell array nor",
               "the internal volume for the mast trunk. Build this, fly it, log it,",
               "then cut moulds."):
        pg.grey(0.25 if ln else 0.0, fill=True)
        pg.text(MARGIN, y, ln, size=3.4)
        y -= 5.0
    y -= 4

    pp = proto_performance()
    st = AN.stability()
    g = AN.geom()
    cg = C.WING["stations"][0][2] + st["cg_target"] * g["mac"] * 1000.0
    rows = [("Span", f"{C.WING['span']:.0f} mm"),
            ("Wing area", f"{g['S'] * 1e4:.0f} cm2"),
            ("Root chord / tip chord",
             f"{C.WING['stations'][0][1]:.0f} / {C.WING['stations'][-1][1]:.0f} mm"),
            ("Aerofoil", f"FF-SC1, {B.wing_section()[2]['t_max'] * 100:.1f} % thick"),
            ("Wing incidence / V-tail", f"+{C.WING['incidence']:.1f} / "
             f"{C.VTAIL['incidence']:.1f} deg"),
            ("Dihedral, outer panels", f"{C.WING['stations'][2][3]:.1f} deg from y="
             f"{C.WING['stations'][1][0]:.0f} mm"),
            ("Washout at tip", f"{abs(C.WING['stations'][-1][4]):.1f} deg"),
            ("CENTRE OF GRAVITY",
             f"{cg:.0f} mm from nose  =  {cg - C.WING['stations'][0][2]:.0f} mm "
             f"aft of the wing leading edge"),
            ("Target all-up mass", f"{pp['mass_kg'] * 1000:.0f} g"),
            ("Predicted stall / best glide",
             f"{pp['v_stall']:.1f} / {pp['v_bg']:.1f} m/s"),
            ("Predicted L/D / min sink",
             f"{pp['ld']:.1f} / {pp['sink_min']:.2f} m/s")]
    pg.grey(0.0, fill=True)
    pg.text(MARGIN, y, "KEY NUMBERS", size=5.0, bold=True)
    y -= 7
    for k, v in rows:
        bold = k.isupper()
        pg.width(0.15).grey(0.85).dash(None)
        pg.line(MARGIN, y - 1.6, W - MARGIN, y - 1.6)
        pg.grey(0.0 if bold else 0.3, fill=True)
        pg.text(MARGIN, y, k, size=3.4, bold=bold)
        pg.grey(0.0, fill=True)
        pg.text(MARGIN + 74, y, v, size=3.4, bold=bold)
        y -= 6.2
    y -= 4

    pg.grey(0.0, fill=True)
    pg.text(MARGIN, y, "SHEET INDEX", size=5.0, bold=True)
    y -= 7
    for name, a, b in sheets:
        pg.grey(0.0, fill=True)
        pg.text(MARGIN, y, f"{a}-{b}" if b > a else f"{a}", size=3.4, bold=True)
        pg.grey(0.25, fill=True)
        pg.text(MARGIN + 16, y, name, size=3.4)
        y -= 5.4

    pg.grey(0.4, fill=True)
    pg.text(MARGIN, MARGIN + 4,
            "Generated by tools/plans.py from tools/config.py. Change a "
            "dimension there and every sheet regenerates.", size=2.8)
    pg.restore()
    return pg


def page_notes(doc, sheet_no, total):
    pg = doc.page(P.A4[0], P.A4[1])
    pg.save().mm_frame()
    W, H = P.A4
    y = H - 22
    pp = proto_performance()

    def head(t):
        nonlocal y
        pg.grey(0.0, fill=True)
        pg.text(MARGIN, y, t, size=5.2, bold=True)
        y -= 7.0

    def body(lines, size=3.3):
        nonlocal y
        for ln in lines:
            pg.grey(0.28 if ln else 0.0, fill=True)
            pg.text(MARGIN, y, ln, size=size)
            y -= 4.8
        y -= 2.5

    pg.grey(0.0, fill=True)
    pg.text(MARGIN, y, "ASSEMBLING THE SHEETS, AND BUILDING FROM THEM",
            size=6.5, bold=True)
    y -= 12

    head("1  TAPE THE TILES")
    body(["Each tile overlaps the next by 12 mm. Trim along the dashed line on",
          "the overlapping sheet only, lay it over its neighbour and line up the",
          "solid triangles, then tape. Work along a row first, then join rows.",
          "The corner cross marks should coincide when you are right."])

    head("2  TWO WAYS TO MAKE THE WING")
    body(["HOT-WIRE CORES (accurate, needs a wire bow). Cut the aerofoil",
          "templates from 1.5 mm ply or aluminium, pin them to the ends of an",
          "XPS block at the right stations, and wire between them. This gives",
          "the true FF-SC1 section and is what the aerofoil sheets are for.",
          "",
          "FLAT FOAM (quick, no tools). Cut the planform from 6 mm depron,",
          "score the underside along the spar line, and fold a thin leading-edge",
          "wrap over a carbon tube. You lose about 15 % of L/D against the true",
          "section but it flies, and the planform and CG are unchanged."])

    head("3  BUILD ORDER")
    body(["Wing first, and weigh every part as you go - the mass budget on the",
          "title sheet is what the predicted performance assumes.",
          "  a  Cut both wing panels. Mirror the template for the left.",
          "  b  Bury the 8 mm carbon spar at 30 % chord, full span, and sleeve",
          "     the joint at the centre.",
          "  c  Set the outer panels up 4.5 deg from the panel break. Block the",
          "     tip 2 deg trailing-edge-up for washout while the glue sets -",
          "     without it the tips stall first and it will drop a wing.",
          "  d  Skin, then cut the aileron and flap free along the hinge lines.",
          "  e  Pod sides, formers, boom. Set the wing seat at +1.5 deg and the",
          "     V-tail at -1.0 deg. That 2.5 deg decalage is the trim.",
          "  f  Fit the gear, then BALANCE before the first flight."])

    head("4  BALANCE - DO NOT SKIP THIS")
    st = AN.stability()
    g = AN.geom()
    cg = C.WING["stations"][0][2] + st["cg_target"] * g["mac"] * 1000.0
    body([f"Centre of gravity {cg:.0f} mm from the nose, which is "
          f"{cg - C.WING['stations'][0][2]:.0f} mm aft of the wing",
          f"leading edge at the root. Marked on the wing and fuselage sheets.",
          "",
          f"That is {st['cg_target'] * 100:.0f} % of mean chord, giving 20 % static",
          f"margin. Forward limit {st['cg_fwd'] * 100:.0f} %, aft limit "
          f"{st['cg_aft'] * 100:.0f} % of mean chord. Nose-heavy is survivable;",
          "tail-heavy is not. Move the battery to trim, and only add lead once",
          "the battery is against a stop."])

    head("5  WHAT TO EXPECT")
    body([f"At {pp['mass_kg'] * 1000:.0f} g the wing loading is "
          f"{pp['wing_loading'] / C.MISSION['g'] * 10:.1f} g/dm2 - lighter than the",
          f"composite aircraft, so it flies slower and thermals better.",
          f"  stall about {pp['v_stall']:.1f} m/s, best glide "
          f"{pp['v_bg']:.1f} m/s, L/D around {pp['ld']:.0f}",
          f"  minimum sink {pp['sink_min']:.2f} m/s at {pp['v_ms']:.1f} m/s",
          "",
          "Hand launch into wind, wings level, firm push. Trim for a flat glide",
          "before touching the throttle."])

    head("6  THEN MEASURE IT")
    body(["Fit the flight controller and an airspeed sensor, fly the polar",
          "sortie in docs/FLIGHT_ANALYSIS.md section 4, and run:",
          "",
          "    python3 tools/flight_analysis.py YOURLOG.BIN --mass 0.80",
          "",
          "That returns the measured sink polar and tells you how the real drag",
          "compares with the prediction above. It is the whole reason to build",
          "this before cutting moulds."])

    frame(pg, "BUILD NOTES", "read before cutting", sheet_no, total)
    pg.restore()
    return pg


# ==========================================================================
# DXF (R12) for CNC hot-wire / laser cutters
# ==========================================================================

def write_dxf(dw, path, kinds=("cut",)):
    out = ["0", "SECTION", "2", "ENTITIES"]
    n = 0
    for it in dw.items:
        if it["t"] != "poly" or it["kind"] not in kinds:
            continue
        out += ["0", "POLYLINE", "8", it["kind"], "66", "1", "70",
                "1" if it["close"] else "0"]
        for (x, y) in it["pts"]:
            out += ["0", "VERTEX", "8", it["kind"],
                    "10", f"{x:.4f}", "20", f"{y:.4f}", "30", "0.0"]
        out += ["0", "SEQEND"]
        n += 1
    out += ["0", "ENDSEC", "0", "EOF"]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").write("\n".join(out) + "\n")
    return n


# ==========================================================================
# main
# ==========================================================================

def main():
    os.makedirs(OUT, exist_ok=True)
    doc = P.Document("FF-1 Ember build plans")

    parts = [
        ("WING PANEL", wing_planform(), True),
        ("AEROFOIL TEMPLATES", None, True),
        ("FUSELAGE POD", fuselage_pod(), True),
        ("FORMERS", formers(), True),
        ("V-TAIL PANEL", vtail_panel(), True),
    ]

    # aerofoil sheets: two templates per landscape page
    st = _wing_stations()
    af_specs = [(st[0][1], "ROOT + PANEL BREAK  (cut 2)"),
                (st[2][1], f"STATION y={st[2][0]:.0f}"),
                (st[3][1], f"STATION y={st[3][0]:.0f}"),
                (st[4][1], "TIP")]

    # --- count pages first so the index and "sheet n of N" are right ---
    def tile_count(dw, pad=6.0):
        pw, ph = P.A4[1], P.A4[0]
        uw, uh = pw - 2 * MARGIN, ph - 2 * MARGIN - 15.0
        x0, y0, x1, y1 = dw.bbox()
        Wd, Hd = (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad
        return (max(1, math.ceil((Wd - OVERLAP) / (uw - OVERLAP)))
                * max(1, math.ceil((Hd - OVERLAP) / (uh - OVERLAP))))

    counts = {}
    for name, dw, _ls in parts:
        counts[name] = 2 if dw is None else tile_count(dw)
    total = 2 + sum(counts.values())

    index, n = [], 2
    for name, dw, _ls in parts:
        index.append((name, n + 1, n + counts[name]))
        n += counts[name]

    page_title(doc, index)
    page_notes(doc, 2, total)

    made = 2
    for name, dw, _ls in parts:
        if dw is None:
            for i in range(0, len(af_specs), 2):
                pg = doc.page(P.A4[1], P.A4[0])
                pg.save().mm_frame(1.0, MARGIN + 14.0, MARGIN + 34.0)
                for j, (c, lab) in enumerate(af_specs[i:i + 2]):
                    d = airfoil_template(c, lab)
                    pg.save().matrix(1, 0, 0, 1, 0, j * 62.0)
                    render(pg, d)
                    pg.restore()
                pg.restore()
                made += 1
                frame(pg, "AEROFOIL TEMPLATES",
                      "FF-SC1 at full size - transfer to 1.5 mm ply or "
                      "aluminium for hot-wire cutting", made, total)
        else:
            pgs = tile(doc, dw, name, made, total)
            made += len(pgs)

    path = os.path.join(OUT, "FF-1_plans_A4.pdf")
    doc.write(path)

    dxf_dir = os.path.join(OUT, "dxf")
    dxf = []
    for name, dw, _ls in parts:
        if dw is None:
            for c, lab in af_specs:
                d = airfoil_template(c, lab)
                f = os.path.join(dxf_dir, f"aerofoil_{c:.0f}mm.dxf")
                dxf.append((f, write_dxf(d, f)))
        else:
            f = os.path.join(dxf_dir, dw.name + ".dxf")
            dxf.append((f, write_dxf(dw, f)))

    print(f"wrote {os.path.relpath(path, ROOT)}  "
          f"({os.path.getsize(path) / 1024:.0f} kB, {made} A4 sheets)")
    for name, a, b in index:
        print(f"   sheets {a:>2}-{b:<2}  {name}")
    print(f"wrote {len(dxf)} DXF outlines to {os.path.relpath(dxf_dir, ROOT)}/")
    pp = proto_performance()
    print(f"prototype: {pp['mass_kg'] * 1000:.0f} g, "
          f"stall {pp['v_stall']:.1f} m/s, L/D {pp['ld']:.1f}, "
          f"min sink {pp['sink_min']:.2f} m/s")
    return path


if __name__ == "__main__":
    main()
