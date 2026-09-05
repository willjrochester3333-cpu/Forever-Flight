"""
Minimal dependency-free triangle mesh kernel for the Forever-Flight airframe.

Pure standard library on purpose: this has to run in a bare container with no
numpy. Meshes here are a few thousand triangles, so speed is a non-issue.

Coordinate system used throughout the project (millimetres):
    +X  aft   (nose at x=0)
    +Y  starboard
    +Z  up
"""

from __future__ import annotations

import math
import struct

Point = tuple


# --------------------------------------------------------------------------
# point transforms -- plain functions (x, y, z) -> (x, y, z)
# --------------------------------------------------------------------------

def translate(dx=0.0, dy=0.0, dz=0.0):
    def f(p):
        return (p[0] + dx, p[1] + dy, p[2] + dz)
    return f


def scale(sx=1.0, sy=None, sz=None):
    sy = sx if sy is None else sy
    sz = sx if sz is None else sz

    def f(p):
        return (p[0] * sx, p[1] * sy, p[2] * sz)
    return f


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))

    def f(p):
        return (p[0], p[1] * c - p[2] * s, p[1] * s + p[2] * c)
    return f


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))

    def f(p):
        return (p[0] * c + p[2] * s, p[1], -p[0] * s + p[2] * c)
    return f


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))

    def f(p):
        return (p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2])
    return f


def compose(*fns):
    """compose(a, b, c)(p) == c(b(a(p))) -- applied left to right."""
    def f(p):
        for fn in fns:
            p = fn(p)
        return p
    return f


def rot_about(deg, axis, origin):
    """Rotate about an axis ('x'|'y'|'z') passing through `origin`."""
    ox, oy, oz = origin
    r = {"x": rot_x, "y": rot_y, "z": rot_z}[axis](deg)
    return compose(translate(-ox, -oy, -oz), r, translate(ox, oy, oz))


# --------------------------------------------------------------------------
# mesh
# --------------------------------------------------------------------------

class Mesh:
    """Indexed triangle soup with named face groups."""

    def __init__(self, name="mesh"):
        self.v: list = []
        self.f: list = []
        self.name = name
        self.groups: list = []  # (name, first_face, last_face_exclusive)

    # -- construction -------------------------------------------------
    def add_vertex(self, p) -> int:
        self.v.append((float(p[0]), float(p[1]), float(p[2])))
        return len(self.v) - 1

    def add_ring(self, pts) -> list:
        return [self.add_vertex(p) for p in pts]

    def tri(self, a, b, c):
        if a != b and b != c and a != c:
            self.f.append((a, b, c))

    def quad(self, a, b, c, d):
        """Quad a-b-c-d, wound so the normal follows the right-hand rule."""
        self.tri(a, b, c)
        self.tri(a, c, d)

    def fan(self, centre_idx, ring, reverse=False):
        n = len(ring)
        for i in range(n):
            a, b = ring[i], ring[(i + 1) % n]
            if reverse:
                self.tri(centre_idx, b, a)
            else:
                self.tri(centre_idx, a, b)

    # -- combination --------------------------------------------------
    def merge(self, other: "Mesh", group_name=None):
        off = len(self.v)
        start = len(self.f)
        self.v.extend(other.v)
        self.f.extend([(a + off, b + off, c + off) for (a, b, c) in other.f])
        if other.groups and group_name is None:
            for (gn, gs, ge) in other.groups:
                self.groups.append((gn, gs + start, ge + start))
        else:
            self.groups.append((group_name or other.name, start, len(self.f)))
        return self

    def copy(self) -> "Mesh":
        m = Mesh(self.name)
        m.v = list(self.v)
        m.f = list(self.f)
        m.groups = list(self.groups)
        return m

    # -- transforms ---------------------------------------------------
    def apply(self, fn) -> "Mesh":
        self.v = [fn(p) for p in self.v]
        return self

    def flip(self) -> "Mesh":
        self.f = [(a, c, b) for (a, b, c) in self.f]
        return self

    def mirrored_y(self, name=None) -> "Mesh":
        """Mirror across the XZ plane; winding is flipped to keep normals out."""
        m = self.copy()
        m.name = name or (self.name + "_port")
        m.apply(lambda p: (p[0], -p[1], p[2]))
        m.flip()
        return m

    # -- measurement --------------------------------------------------
    def bounds(self):
        xs = [p[0] for p in self.v]
        ys = [p[1] for p in self.v]
        zs = [p[2] for p in self.v]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def volume_mm3(self):
        """Signed volume via the divergence theorem; only meaningful if closed."""
        tot = 0.0
        for (a, b, c) in self.f:
            ax, ay, az = self.v[a]
            bx, by, bz = self.v[b]
            cx, cy, cz = self.v[c]
            tot += (ax * (by * cz - bz * cy)
                    - ay * (bx * cz - bz * cx)
                    + az * (bx * cy - by * cx))
        return tot / 6.0

    def area_mm2(self):
        tot = 0.0
        for (a, b, c) in self.f:
            ax, ay, az = self.v[a]
            bx, by, bz = self.v[b]
            cx, cy, cz = self.v[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            tot += 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz)
        return tot

    def edge_report(self):
        """Count edges by how many faces use them. A closed manifold is all 2s."""
        seen = {}
        for (a, b, c) in self.f:
            for e in ((a, b), (b, c), (c, a)):
                k = (min(e), max(e))
                seen[k] = seen.get(k, 0) + 1
        hist = {}
        for n in seen.values():
            hist[n] = hist.get(n, 0) + 1
        return hist


# --------------------------------------------------------------------------
# lofting
# --------------------------------------------------------------------------

def loft(sections, closed_rings=True, cap_start=True, cap_end=True, name="loft"):
    """Skin a sequence of equal-length point rings.

    `sections` is a list of rings; every ring must hold the same number of
    points, ordered consistently, or the skin will twist.
    """
    m = Mesh(name)
    if not sections:
        return m
    n = len(sections[0])
    for s in sections:
        if len(s) != n:
            raise ValueError("all loft sections need the same point count")

    rings = [m.add_ring(s) for s in sections]

    for i in range(len(rings) - 1):
        r0, r1 = rings[i], rings[i + 1]
        last = n if closed_rings else n - 1
        for j in range(last):
            k = (j + 1) % n
            m.quad(r0[j], r1[j], r1[k], r0[k])

    if cap_start:
        _cap(m, sections[0], rings[0], reverse=True)
    if cap_end:
        _cap(m, sections[-1], rings[-1], reverse=False)
    return m


def _cap(m: Mesh, pts, ring, reverse):
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    cz = sum(p[2] for p in pts) / len(pts)
    ci = m.add_vertex((cx, cy, cz))
    m.fan(ci, ring, reverse=reverse)


def superellipse(width, height, n=3.0, npts=32, cz=0.0):
    """Rounded-rectangle cross-section in the YZ plane. n=2 -> ellipse."""
    pts = []
    a, b = width / 2.0, height / 2.0
    for i in range(npts):
        t = 2.0 * math.pi * i / npts
        ct, st = math.cos(t), math.sin(t)
        y = a * math.copysign(abs(ct) ** (2.0 / n), ct)
        z = b * math.copysign(abs(st) ** (2.0 / n), st)
        pts.append((0.0, y, z + cz))
    return pts


def circle(r, npts=24, cz=0.0):
    return [(0.0,
             r * math.cos(2 * math.pi * i / npts),
             r * math.sin(2 * math.pi * i / npts) + cz)
            for i in range(npts)]


# --------------------------------------------------------------------------
# writers
# --------------------------------------------------------------------------

def write_stl(mesh: Mesh, path, header="Forever-Flight"):
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii", "replace")[:80].ljust(80, b" "))
        fh.write(struct.pack("<I", len(mesh.f)))
        for (a, b, c) in mesh.f:
            ax, ay, az = mesh.v[a]
            bx, by, bz = mesh.v[b]
            cx, cy, cz = mesh.v[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            fh.write(struct.pack("<12fH",
                                 nx / ln, ny / ln, nz / ln,
                                 ax, ay, az, bx, by, bz, cx, cy, cz, 0))
    return path


def write_obj(mesh: Mesh, path, name="forever_flight"):
    with open(path, "w") as fh:
        fh.write(f"# {name} -- generated by tools/build_model.py\n")
        fh.write(f"# {len(mesh.v)} vertices, {len(mesh.f)} triangles\n")
        fh.write("# units: millimetres, +X aft, +Y starboard, +Z up\n")
        for (x, y, z) in mesh.v:
            fh.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        spans = sorted(mesh.groups, key=lambda g: g[1]) or [("all", 0, len(mesh.f))]
        for (gname, gs, ge) in spans:
            fh.write(f"g {gname}\n")
            for (a, b, c) in mesh.f[gs:ge]:
                fh.write(f"f {a + 1} {b + 1} {c + 1}\n")
    return path
