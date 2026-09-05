#!/usr/bin/env python3
"""
Dependency-free orthographic renderer: z-buffered flat shading straight to PNG.

Exists so the geometry can be eyeballed in a container with no graphics stack.
Writes three-views and an isometric of whatever build_model.py produced.
"""

from __future__ import annotations

import math
import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import build_model as B
import mesh as M

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "img")


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))
    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)


PALETTE = {
    "wing_stbd": (214, 216, 219), "wing_port": (214, 216, 219),
    "fuselage": (198, 201, 205), "dorsal_spine": (176, 180, 186),
    "ventral_fairing": (176, 180, 186), "sensor_turret": (74, 78, 84),
    "vtail_stbd": (206, 209, 213), "vtail_port": (206, 209, 213),
    "motor_pod": (196, 92, 60), "rat_pod": (58, 122, 148),
    "solar_cells": (28, 34, 46),
}


def face_colours(mesh: M.Mesh, default=(200, 203, 208)):
    cols = [default] * len(mesh.f)
    for (name, gs, ge) in mesh.groups:
        c = PALETTE.get(name)
        if c:
            for i in range(gs, ge):
                cols[i] = c
    return cols


def render(mesh: M.Mesh, view, w=1100, h=760, bg=(250, 249, 247), path="out.png",
           light=(-0.42, -0.50, 0.76), base=(200, 203, 208), pad=0.06):
    """view: point -> (screen_x, screen_y, depth), all in model units."""
    pts = [view(p) for p in mesh.v]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    sx = (w * (1 - 2 * pad)) / max(max(xs) - min(xs), 1e-6)
    sy = (h * (1 - 2 * pad)) / max(max(ys) - min(ys), 1e-6)
    s = min(sx, sy)
    ox = w / 2 - s * (max(xs) + min(xs)) / 2
    oy = h / 2 + s * (max(ys) + min(ys)) / 2
    scr = [(s * p[0] + ox, oy - s * p[1], p[2]) for p in pts]

    buf = bytearray()
    for _ in range(w * h):
        buf += bytes(bg)
    zb = [1e30] * (w * h)

    ln = math.sqrt(sum(c * c for c in light)) or 1.0
    lx, ly, lz = (c / ln for c in light)
    cols = face_colours(mesh, base)

    for fi, (a, b, c) in enumerate(mesh.f):
        ax, ay, az = mesh.v[a]; bx, by, bz = mesh.v[b]; cx, cy, cz = mesh.v[c]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        nl = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nl < 1e-12:
            continue
        d = (nx * lx + ny * ly + nz * lz) / nl
        sh = 0.52 + 0.58 * max(0.0, d) + 0.20 * max(0.0, -d)
        col = tuple(min(255, int(v * sh)) for v in cols[fi])

        p0, p1, p2 = scr[a], scr[b], scr[c]
        minx = max(0, int(min(p0[0], p1[0], p2[0])))
        maxx = min(w - 1, int(max(p0[0], p1[0], p2[0])) + 1)
        miny = max(0, int(min(p0[1], p1[1], p2[1])))
        maxy = min(h - 1, int(max(p0[1], p1[1], p2[1])) + 1)
        if minx > maxx or miny > maxy:
            continue
        den = ((p1[1] - p2[1]) * (p0[0] - p2[0]) + (p2[0] - p1[0]) * (p0[1] - p2[1]))
        if abs(den) < 1e-9:
            continue
        for py in range(miny, maxy + 1):
            for px in range(minx, maxx + 1):
                fx, fy = px + 0.5, py + 0.5
                w0 = ((p1[1] - p2[1]) * (fx - p2[0]) + (p2[0] - p1[0]) * (fy - p2[1])) / den
                w1 = ((p2[1] - p0[1]) * (fx - p2[0]) + (p0[0] - p2[0]) * (fy - p2[1])) / den
                w2 = 1.0 - w0 - w1
                if w0 < 0 or w1 < 0 or w2 < 0:
                    continue
                z = w0 * p0[2] + w1 * p1[2] + w2 * p2[2]
                i = py * w + px
                if z < zb[i]:
                    zb[i] = z
                    buf[i * 3:i * 3 + 3] = bytes(col)
    write_png(path, w, h, buf)
    return path


def iso(az_deg, el_deg):
    az, el = math.radians(az_deg), math.radians(el_deg)

    def f(p):
        x, y, z = p
        xr = x * math.cos(az) - y * math.sin(az)
        yr = x * math.sin(az) + y * math.cos(az)
        return (xr, z * math.cos(el) - yr * math.sin(el),
                yr * math.cos(el) + z * math.sin(el))
    return f


def main():
    os.makedirs(OUT, exist_ok=True)
    dep, _, _, _ = B.assemble(True)
    stow, _, _, _ = B.assemble(False)
    jobs = [
        (dep,  lambda p: (p[0], p[1], -p[2]),  "plan_deployed.png",   1200, 820),
        (dep,  lambda p: (p[0], p[2], p[1]),   "side_deployed.png",   1200, 620),
        (stow, lambda p: (p[0], p[2], p[1]),   "side_stowed.png",     1200, 620),
        (dep,  lambda p: (p[1], p[2], p[0]),   "front_deployed.png",  1100, 620),
        (dep,  iso(35, -20),                   "iso_deployed.png",    1300, 860),
        (stow, iso(35, -20),                   "iso_stowed.png",      1300, 860),
        (dep,  iso(150, 16),                   "iso_underside.png",   1300, 860),
    ]
    for (m, v, name, w, h) in jobs:
        p = render(m, v, w=w, h=h, path=os.path.join(OUT, name))
        print("wrote", os.path.relpath(p, ROOT), f"({os.path.getsize(p)//1024} kB)")


if __name__ == "__main__":
    main()
