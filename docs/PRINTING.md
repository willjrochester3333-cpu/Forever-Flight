# Printing it on a Bambu Lab A2L

```
python3 blender/print_parts.py      cuts the model up and writes print/*.stl
python3 blender/check_parts.py      audits the STLs before you print them
python3 tools/spar.py               the spar sizing behind the bore diameters
```

![parts](img/blender/parts-blend.png)

Open **`blender/jetwing-parts.blend`** to browse them: every part in print
orientation, named and labelled with its size. Or go straight to the STLs in
`print/`.

**30 parts, every one inside 330 × 320 × 325 mm, every one watertight, none
self-intersecting.**

## What the A2L changes

It is not the plate area that matters, it is the **height**: 325 mm against
the A1's 256. Wing sections print standing on end, so height is span, and
span is what sets the part count.

| | A1 (256³) | A2L (330 × 320 × 325) |
|---|---|---|
| wing sections per side | 5 at 206 mm | **4 at 257.5 mm** |
| root section | split chordwise as well — 260 mm chord on a 256 mm bed | **one piece, square on the plate** |
| winglets | not printable at all | **lofted onto the outboard section** |
| ailerons per side | 3 | **2** |
| flaps per side | 2 | 2 |
| fuselage | 4 rings of 174 mm | **3 rings of 232 mm** |
| parts | 37 | **30** |
| glued joints | 19 | **12** |

Three sections a side would be better still and does not go: 1030 mm of
semi-span in three is 343 mm, and the plate is 325.

The root section is the one that changes character. On the A1 a 260 mm root
chord did not fit a 256 mm bed at all, so it went on the diagonal and then had
to be split chordwise as well. On the A2L it stands square with 60 mm to
spare, as one piece.

### The winglets are new

They were never in the printable set. `jetwing.py` lofts them onto the tip
*inside* the Wing object, and the print sections are generated from the
planform rather than cut out of that object — so nothing ever emitted them.
The aero model had 150 mm winglets and the box had none.

They stay **attached** to the outboard section rather than becoming parts of
their own, and not only because there is room. A winglet is a cantilever in
side load and the tip section is 11 mm thick, so a glued butt joint there is
the worst joint on the aircraft and there is nothing to pin into. Lofted on,
there is no joint to fail. `wing_R4` is 280 × 304 × 159 mm with the winglet
on it.

## How the parts sit

Wing sections print **standing on end, span axis vertical**. That:

- puts the spar bore straight up Z, so it needs no support and comes out round
- lays the layer lines across the chord, where the skin wants them
- makes the limit **325 mm of span**, not of chord

`wing_R4` is the exception worth knowing about: rotating its span up to Z lays
the winglet flat across the plate, which is how a part with a 150 mm winglet
on it fits in 304 mm of height.

Nothing is placed diagonally. The A1 cut *needed* that, and a part that only
fits cornerwise is a part a slicer can quietly place wrong.

## The carbon

Sized in `tools/spar.py` from the actual span loading, not a rule of thumb:
**19.8 N·m of root bending at 9 g ultimate** (6 g limit × 1.5), with the
lift distribution taken from the lifting-line solution for this planform.

| | | |
|---|---|---|
| **Spar A** | 10 × 8 mm tube, **900 mm** | root spar *and* wing joiner, centred |
| **Spar B** | 8 × 6 mm tube, **500 mm**, one per side | inserts 70 mm inside A |
| **Anti-rotation** | 4 mm rod, **380 mm** | across the centre at 62% chord |
| **Joint dowels** | 4 mm rod, **22 mm** each, 12 off | two per joint face |

Spar B is 8 mm OD and Spar A is 8 mm ID, so B **telescopes inside A** — that
is why those two sizes and not any other pair. Bores are cut at **+0.4 mm on
diameter** for a sliding fit after shrinkage; if your printer runs tight,
open them with a drill rather than reprinting.

Why it steps down: the wing gets thin outboard. At the 106 mm tip and 10.5%
thickness there is 11.1 mm of section to put a rod in, and a 10 mm rod leaves
no wall at all. Strength is not the binding constraint past mid-span — room
is. The moment at 80% span is 0.3 N·m, which a 4 mm rod carries easily.

**The anti-rotation rod is not optional.** One round tube, however well
fitted, lets the panels rotate about it, and a wing panel at the wrong
incidence is a wing panel that does not fly.

## Joints

Each joint face carries **two 4 mm dowel holes, 11 mm deep** — up from 3 × 7
on the A1, because the sections are half again as long and the joint carries
more bending. They sit at 14% and 60% chord, either side of the spar at 30%,
so a section cannot rotate while the glue goes off.

Both holes run **parallel to the spar**, not along their own chord lines. The
14, 30 and 60 per cent chord lines each have their own sweep angle: over a
22 mm dowel the 60% line diverges from the spar by 0.49 mm, which is more than
the fit clearance, so three bores each on its own axis would simply refuse to
go together.

| joint | section at 14% c | at 60% c | wall round a 4.4 mm bore |
|---|---|---|---|
| y = 257.5 | 20.3 mm | 17.7 mm | 6.6–8.0 mm |
| y = 515 | 16.8 mm | 14.6 mm | 5.1–6.2 mm |
| y = 772.5 | 13.3 mm | 11.5 mm | 3.6–4.4 mm |

The aft hole used to be specified at 52% chord, which is exactly where the
servo loom channel runs. A 5 mm bore swallows a 3.4 mm dowel hole whole, so
on every joint inboard of y = 620 it would have located nothing. (It would
also have located nothing because the holes were never actually cut — see
below.)

The fuselage rings butt without dowels, and do not need them: the section is
a 96 × 104 rounded superellipse rather than a circle, so each joint keys
itself. There is one way the rings go together and it is obvious by feel.

## The servo loom

A **6 mm channel** runs spanwise at 50% chord from the aileron bay in to the
root, straight through every joint, so you thread the wiring rather than fish
it. It passes deliberately *through* both servo bays, which is what the servo
lead has to do anyway.

It is **swept** along the mid-thickness line, not driven straight between the
ends of each section. The wing is swept and tapered, so the 50% chord line is
not parallel to anything: a single straight cutter across a 257 mm section
sits on 50% chord at both joints and wanders **11.3 mm** off it in the middle.
Swept, it stays mid-section the whole way and meets the next section's channel
with a measured step of **0.000 mm** on a 6 mm bore.

## Parts

```
part                       X       Y       Z   faces  fits A2L?  watertight?
  aileron_L1           105.5   203.0    16.0     802      yes      yes
  aileron_L2            97.7   203.0    14.1     802      yes      yes
  aileron_R1           105.5   203.0    16.0     802      yes      yes
  aileron_R2            97.7   203.0    14.1     802      yes      yes
  flap_L1              115.1   182.4    19.1     802      yes      yes
  flap_L2              108.0   182.4    17.5     802      yes      yes
  flap_R1              115.1   182.4    19.1     802      yes      yes
  flap_R2              108.0   182.4    17.5     802      yes      yes
  fuselage_1           232.0    98.4   108.8     196      yes      yes
  fuselage_2           232.0    98.4   111.9     196      yes      yes
  fuselage_3           232.0    77.8    83.1     170      yes      yes
  hatch_aileron_L       44.1    32.3     4.8      88      yes      yes
  hatch_aileron_R       44.1    32.3     4.8      88      yes      yes
  hatch_flap_L          44.1    32.3     4.5      88      yes      yes
  hatch_flap_R          44.1    32.3     4.5      88      yes      yes
  part_Ducta           197.0    52.0    66.0     171      yes      yes
  part_Ductb           197.0    52.0    64.4     118      yes      yes
  part_Fin             186.7    14.5   202.0     415      yes      yes
  part_Inlet            99.4   125.5    38.5     440      yes      yes
  part_Motor            78.9    26.0    27.6      86      yes      yes
  part_Rudder          139.7    10.6   227.9     422      yes      yes
  part_Stator           21.9    47.5    47.9      98      yes      yes
  wing_L1              303.5   257.5    34.1    1640      yes      yes
  wing_L2              250.1   257.5    30.2    1740      yes      yes
  wing_L3              221.6   257.5    26.3    1587      yes      yes
  wing_L4              279.6   303.9   159.4    2276      yes      yes
  wing_R1              303.5   257.5    34.1    1640      yes      yes
  wing_R2              250.1   257.5    30.2    1740      yes      yes
  wing_R3              221.6   257.5    26.3    1587      yes      yes
  wing_R4              279.6   303.9   159.4    2276      yes      yes

  30 parts, 0 too big for 330x320x325, 0 not watertight
  written to /home/user/Forever-Flight/print/
```

## Check before you print

`blender/check_parts.py` loads every exported STL back in and measures three
things: positive volume in the right ballpark, no non-manifold edges, and **no
two faces passing through each other**.

```
  30 parts, 5553 cm3 of enclosed volume (not filament: the slicer
  hollows these out)
  all parts manifold, positive volume, no self-intersections
```

That last test exists because of what it found. `print_parts.py` had always
checked watertightness and bed fit, and neither catches the failure that
actually occurred: **two wing sections were not wing sections.**

Blender's exact boolean solver has no failure return. Handed a target it
cannot reason about it produces something plausible instead, and here it
returned the *cutter* in place of the part. A 6 mm tube is watertight, it
fits the bed and it slices — so it passed every check and went in the bundle.

The cause was upstream. `section_cut` built the closing strip across the hinge
from the upper surface to the lower one, but appended it *after* the outline
had already arrived at the lower surface. The strip therefore retraced the
hinge line twice and the lofted body intersected itself: **338 self-
intersecting face pairs per section**, invisible in a render, and enough to
make every boolean on the part a coin flip. The A1 cut carried the same bug
and happened to get the right answer.

Three more of the same kind came out of the same audit:

- **Flaps and ailerons were the wrong shape.** They were built by clamping the
  full section's chord into [hinge, 1], which piles every point ahead of the
  hinge onto the hinge line *keeping its own thickness* — so each one carried
  a zero-width flap of surface standing proud of its own hinge face, right up
  to maximum thickness. They measured 30.0 mm deep where the aerofoil says
  19.1, and would have fouled the wing cut-out. Normals inside-out with it,
  and 1,477 self-intersecting pairs in one aileron.
- **The rudder had the same clamp**, from `jetwing.py` this time: 314 pairs,
  and 14.4 mm thick where it should be 10.6.
- **The wing joints had no dowel holes.** The only caller of the function that
  cut them was a spanwise splitter that stopped being used when the sections
  started being generated instead of cut. The docs specified the holes; the
  STLs did not have them.

`boolean()` now checks its own result — a difference cannot halve a face count
or shrink a bounding box — so a silent inversion raises instead of shipping.

## Slicer

For LW-PLA on an A2L, as a starting point rather than gospel:

| | |
|---|---|
| nozzle / layer | 0.4 mm / 0.2 mm |
| walls | 2 perimeters on wing sections, 3 on the fuselage |
| infill | 4–6% gyroid in the wing, 0% in the fuselage shells |
| top/bottom | 0 on wing sections — the perimeters are the skin |
| supports | none needed in these orientations |

The fuselage, duct and inlets are exported as **shells with a real wall**
(1.0–1.2 mm) rather than solid blocks, so the slicer is not hollowing out
something it then has to fill back in.

`wing_R1` encloses a litre. At these settings that is not a litre of filament,
but it is a long print — start it when you have the machine free.

## Assembly order

The full build manual, with the CG, throws and pre-flight checks, is in
[ASSEMBLY.md](ASSEMBLY.md). In short:

1. Thread **Spar A** through both root sections, dry, and check the panels
   sit at the same incidence before any glue.
2. Slide **Spar B** into A from outboard, through sections 2 and 3.
3. Thread the **servo loom** through the 6 mm channel before bonding anything
   — it runs root to y = 620 and you cannot get at it afterwards.
4. Dowel and bond the wing sections outward from the root, one joint at a
   time, checking the tip stays in line. Three joints a side, not five.
5. Fit the **anti-rotation rod** last — it will only go in if the panels are
   already correctly aligned, which is the point of it.
6. Fuselage rings bond nose-to-tail; the duct goes in before the last ring
   closes it up.

## How the parts are made

They are **generated from the parametric loft**, not cut out of the model.
Cutting was tried first and produced nonsense: the wing object is several
lofted runs merged into one mesh, so where two runs abut there are coincident
internal faces, and an exact boolean on a non-manifold mesh returns garbage —
sections came out with the wrong span and chunks missing.

Generating each section from the same maths the skin is made of is manifold
by construction and lands on the surface exactly. The spar bores, servo bays,
pushrod guides, loom channel and dowel holes are then boolean-subtracted,
which works because the sections are clean.

Where a section sits behind a control surface the aft part of the aerofoil
has to stop at the hinge, and the section is **resampled** to the hinge and
closed with a single deliberate edge. Not clamped — clamping is what produced
the wrong-shaped flaps above. And a single edge, with no interior points along
it: points spread along a closing strip are exactly collinear, and a cap
n-gon with a collinear run triangulates into zero-area slivers, which are
self-intersections by another name. Those were the last ones in the bundle.
