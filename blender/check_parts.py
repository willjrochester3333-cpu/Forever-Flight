#!/usr/bin/env python3
"""
Audit the exported STLs. Run it after print_parts.py, before printing anything.

    python3 blender/check_parts.py
    blender --background --python blender/check_parts.py

WHY THIS EXISTS
    print_parts.py already checks that each part fits the bed and is
    watertight. Neither catches the failure that actually happened.

    Two of the wing sections were not wing sections. Blender's exact boolean
    solver has no failure return: handed a target it cannot reason about it
    produces something plausible instead, and here it returned the CUTTER in
    place of the part. A 6 mm tube is watertight, it fits the bed, and it
    slices -- so it passed every check and went in the bundle.

    The cause was a self-intersecting target: section_cut built the closing
    strip from the upper surface to the lower one, but appended it after the
    outline had already reached the lower surface, so the strip retraced the
    hinge line twice. 338 self-intersecting face pairs per section, invisible
    in a render, fatal to a boolean.

    So this checks the three things that would have caught it:

      volume      positive, and in the right ballpark. A part replaced by a
                  cutter has the wrong volume by an order of magnitude, and a
                  part with inverted normals has a negative one.
      manifold    no open or shared-by-three edges.
      self-x      no two faces passing through each other. This is the one
                  that matters, because it is the upstream cause rather than
                  a symptom, and nothing else in the pipeline looks for it.
"""

import os
import sys

import bpy
import bmesh
from mathutils.bvhtree import BVHTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import print_parts as PP

# A part this small is a fragment of something, not a part. The smallest real
# part is the stator at about 2 cm3.
MIN_CM3 = 0.5


def audit(path):
    """Load one STL and measure it. Returns (volume cm3, faces, non-manifold,
    self-intersecting pairs)."""
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=path)
    ob = next(iter(set(bpy.data.objects) - before))

    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    # the STLs are written at global_scale 1000, so these coordinates are
    # millimetres and the volume comes out in cubic millimetres
    vol = bm.calc_volume(signed=True) / 1000.0
    nonman = sum(1 for e in bm.edges if not e.is_manifold)
    loose = sum(1 for v in bm.verts if not v.link_faces)
    verts = [v.co.copy() for v in bm.verts]
    polys = [[v.index for v in f.verts] for f in bm.faces]
    bm.free()
    bpy.data.objects.remove(ob, do_unlink=True)

    tree = BVHTree.FromPolygons(verts, polys, all_triangles=True, epsilon=0.0)
    selfx = 0
    for (i, j) in tree.overlap(tree):
        if i >= j:
            continue
        if set(polys[i]) & set(polys[j]):
            continue            # neighbours, which share a vertex by design
        selfx += 1
    return vol, len(polys), nonman + loose, selfx


def main():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)

    if not os.path.isdir(PP.OUT):
        print("  no %s/ -- run blender/print_parts.py first" % PP.OUT)
        return 1
    names = sorted(f for f in os.listdir(PP.OUT) if f.endswith(".stl"))
    if not names:
        print("  no STLs in %s/ -- run blender/print_parts.py first" % PP.OUT)
        return 1

    print("\n  %-20s %9s %7s %8s %7s" %
          ("part", "cm3", "faces", "manifold", "self-x"))
    bad, total = [], 0.0
    for f in names:
        vol, nf, nonman, selfx = audit(os.path.join(PP.OUT, f))
        why = []
        if vol < MIN_CM3:
            why.append("volume %+.2f cm3" % vol)
        if nonman:
            why.append("%d bad edges" % nonman)
        if selfx:
            why.append("%d self-intersections" % selfx)
        total += max(vol, 0.0)
        print("  %-20s %9.1f %7d %8s %7d%s"
              % (f[:-4], vol, nf, "yes" if not nonman else "NO", selfx,
                 "   <-- " + ", ".join(why) if why else ""))
        if why:
            bad.append((f[:-4], ", ".join(why)))

    print("\n  %d parts, %.0f cm3 of enclosed volume (not filament: the slicer\n  hollows these out)" % (len(names), total))
    if bad:
        print("  %d PARTS WILL NOT PRINT RELIABLY:" % len(bad))
        for (nm, why) in bad:
            print("     %-20s %s" % (nm, why))
        return 1
    print("  all parts manifold, positive volume, no self-intersections")
    return 0


if __name__ == "__main__":
    sys.exit(main())
