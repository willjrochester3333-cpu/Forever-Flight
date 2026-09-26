"""
Minimal vector PDF writer. Standard library only.

Enough of PDF 1.4 to emit multi-page A4 line drawings at an exact scale:
paths, dashes, line widths, greys, text in the base-14 Helvetica (no font
embedding needed), and a transformation matrix so the caller can draw in
millimetres and let the writer map to points.

Why not SVG or PNG: a printed template has to come out the right SIZE. PDF
carries an explicit MediaBox in points, so "actual size" printing is exact and
reproducible; a browser printing an SVG is not.
"""

from __future__ import annotations

import zlib

MM = 72.0 / 25.4          # points per millimetre
A4 = (210.0, 297.0)       # mm


def esc(s):
    return (str(s).replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)"))


class Page:
    """A page's content stream, addressed in millimetres."""

    def __init__(self, w_mm=A4[0], h_mm=A4[1]):
        self.w, self.h = w_mm, h_mm
        self.ops = []
        self._stack = 0

    # -- state -------------------------------------------------------
    def save(self):
        self.ops.append("q")
        self._stack += 1
        return self

    def restore(self):
        self.ops.append("Q")
        self._stack -= 1
        return self

    def matrix(self, a, b, c, d, e, f):
        self.ops.append(f"{a:.6f} {b:.6f} {c:.6f} {d:.6f} {e:.4f} {f:.4f} cm")
        return self

    def mm_frame(self, sx=1.0, ox_mm=0.0, oy_mm=0.0):
        """Draw in mm from here on, scaled by sx, origin at (ox,oy) mm."""
        return self.matrix(MM * sx, 0, 0, MM * sx, ox_mm * MM, oy_mm * MM)

    def clip_rect(self, x, y, w, h):
        self.ops.append(f"{x:.3f} {y:.3f} {w:.3f} {h:.3f} re W n")
        return self

    def width(self, w_mm):
        self.ops.append(f"{w_mm:.3f} w")
        return self

    def dash(self, pattern=None, phase=0.0):
        if not pattern:
            self.ops.append("[] 0 d")
        else:
            self.ops.append("[" + " ".join(f"{p:.2f}" for p in pattern)
                            + f"] {phase:.2f} d")
        return self

    def grey(self, g, fill=False):
        self.ops.append(f"{g:.3f} {'g' if fill else 'G'}")
        return self

    def rgb(self, r, g, b, fill=False):
        self.ops.append(f"{r:.3f} {g:.3f} {b:.3f} {'rg' if fill else 'RG'}")
        return self

    def cap(self, style=1):
        self.ops.append(f"{style} J")
        return self

    # -- geometry ----------------------------------------------------
    def poly(self, pts, close=False, stroke=True, fill=False):
        if len(pts) < 2:
            return self
        self.ops.append(f"{pts[0][0]:.4f} {pts[0][1]:.4f} m")
        for x, y in pts[1:]:
            self.ops.append(f"{x:.4f} {y:.4f} l")
        if close:
            self.ops.append("h")
        self.ops.append("B" if (fill and stroke) else "f" if fill else "S")
        return self

    def line(self, x0, y0, x1, y1):
        self.ops.append(f"{x0:.4f} {y0:.4f} m {x1:.4f} {y1:.4f} l S")
        return self

    def rect(self, x, y, w, h, stroke=True, fill=False):
        self.ops.append(f"{x:.4f} {y:.4f} {w:.4f} {h:.4f} re "
                        + ("B" if (fill and stroke) else "f" if fill else "S"))
        return self

    def circle(self, cx, cy, r, stroke=True, fill=False):
        k = 0.5523 * r
        self.ops.append(f"{cx - r:.4f} {cy:.4f} m")
        self.ops.append(f"{cx - r:.4f} {cy + k:.4f} {cx - k:.4f} {cy + r:.4f} "
                        f"{cx:.4f} {cy + r:.4f} c")
        self.ops.append(f"{cx + k:.4f} {cy + r:.4f} {cx + r:.4f} {cy + k:.4f} "
                        f"{cx + r:.4f} {cy:.4f} c")
        self.ops.append(f"{cx + r:.4f} {cy - k:.4f} {cx + k:.4f} {cy - r:.4f} "
                        f"{cx:.4f} {cy - r:.4f} c")
        self.ops.append(f"{cx - k:.4f} {cy - r:.4f} {cx - r:.4f} {cy - k:.4f} "
                        f"{cx - r:.4f} {cy:.4f} c")
        self.ops.append("B" if (fill and stroke) else "f" if fill else "S")
        return self

    def cross(self, cx, cy, r=3.0):
        return self.line(cx - r, cy, cx + r, cy).line(cx, cy - r, cx, cy + r)

    # -- text --------------------------------------------------------
    def text(self, x, y, s, size=3.0, bold=False, rot=0.0, align="l"):
        """size is in mm of cap-ish height; text is placed in the current frame."""
        font = "F2" if bold else "F1"
        w = self.text_width(s, size, bold)
        if align == "c":
            x -= w / 2.0
        elif align == "r":
            x -= w
        self.ops.append("BT")
        self.ops.append(f"/{font} {size:.3f} Tf")
        if rot:
            import math
            c, sn = math.cos(math.radians(rot)), math.sin(math.radians(rot))
            self.ops.append(f"{c:.6f} {sn:.6f} {-sn:.6f} {c:.6f} "
                            f"{x:.4f} {y:.4f} Tm")
        else:
            self.ops.append(f"1 0 0 1 {x:.4f} {y:.4f} Tm")
        self.ops.append(f"({esc(s)}) Tj")
        self.ops.append("ET")
        return self

    @staticmethod
    def text_width(s, size, bold=False):
        # Helvetica average advance; good enough for centring labels.
        f = 0.56 if bold else 0.52
        return len(str(s)) * size * f

    def stream(self):
        while self._stack > 0:
            self.restore()
        return "\n".join(self.ops).encode("latin-1", "replace")


class Document:
    def __init__(self, title="Plans", compress=True):
        self.pages = []
        self.title = title
        self.compress = compress

    def page(self, w_mm=A4[0], h_mm=A4[1]):
        p = Page(w_mm, h_mm)
        self.pages.append(p)
        return p

    def write(self, path):
        objs = []                                    # 1-indexed bodies

        def add(body):
            objs.append(body)
            return len(objs)

        font_r = add(b"<</Type/Font/Subtype/Type1/BaseFont/Helvetica"
                     b"/Encoding/WinAnsiEncoding>>")
        font_b = add(b"<</Type/Font/Subtype/Type1/BaseFont/Helvetica-Bold"
                     b"/Encoding/WinAnsiEncoding>>")
        pages_id = add(b"")                          # placeholder, filled below

        kids = []
        for pg in self.pages:
            data = pg.stream()
            if self.compress:
                body = (b"<</Length " + str(len(zlib.compress(data, 9))).encode()
                        + b"/Filter/FlateDecode>>stream\n"
                        + zlib.compress(data, 9) + b"\nendstream")
            else:
                body = (b"<</Length " + str(len(data)).encode() + b">>stream\n"
                        + data + b"\nendstream")
            cid = add(body)
            pid = add(
                b"<</Type/Page/Parent " + str(pages_id).encode() + b" 0 R"
                + f"/MediaBox[0 0 {pg.w * MM:.3f} {pg.h * MM:.3f}]".encode()
                + b"/Resources<</Font<</F1 " + str(font_r).encode()
                + b" 0 R/F2 " + str(font_b).encode() + b" 0 R>>>>"
                + b"/Contents " + str(cid).encode() + b" 0 R>>")
            kids.append(pid)

        objs[pages_id - 1] = (
            b"<</Type/Pages/Count " + str(len(kids)).encode() + b"/Kids["
            + b" ".join(str(k).encode() + b" 0 R" for k in kids) + b"]>>")
        info = add(b"<</Title(" + esc(self.title).encode("latin-1", "replace")
                   + b")/Producer(FF-1 plans.py)>>")
        root = add(b"<</Type/Catalog/Pages " + str(pages_id).encode() + b" 0 R>>")

        out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
        offsets = [0]
        for i, body in enumerate(objs, start=1):
            offsets.append(len(out))
            out += str(i).encode() + b" 0 obj\n" + body + b"\nendobj\n"
        xref_at = len(out)
        out += b"xref\n0 " + str(len(objs) + 1).encode() + b"\n"
        out += b"0000000000 65535 f \n"
        for off in offsets[1:]:
            out += f"{off:010d} 00000 n \n".encode()
        out += (b"trailer\n<</Size " + str(len(objs) + 1).encode()
                + b"/Root " + str(root).encode() + b" 0 R"
                + b"/Info " + str(info).encode() + b" 0 R>>\n"
                + b"startxref\n" + str(xref_at).encode() + b"\n%%EOF\n")
        open(path, "wb").write(bytes(out))
        return path
