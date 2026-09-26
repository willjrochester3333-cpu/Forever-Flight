# Printable build plans

```bash
python3 tools/plans.py
```

Writes `plans/FF-1_plans_A4.pdf` — 22 A4 sheets at exact full size — plus
`plans/dxf/` outlines for anyone with a CNC hot-wire or laser cutter.

## Print settings, and why they matter

**100 % / "Actual size". Turn off "Fit to page", "Shrink oversized pages" and
"Scale to fit".**

Those options are on by default in most print dialogs and take 4–6 % off. On
this aircraft a 5 % shrink moves the centre of gravity 3 mm forward of where
the plan marks it and shortens every chord, so the wing you cut is not the
wing that was designed. Every sheet carries a 100 mm bar — measure it with a
steel rule before you cut anything.

The PDF's page box is exactly 595.276 × 841.890 pt (210 × 297 mm) and the
drawing transform is 2.834646 pt/mm, so 100 drawing units land at 100.00001 mm
on paper. Any error you measure is the printer, not the file.

## What it is, and what it is not

A **foam prototype of the FF-1's aerodynamic shape** — wing, V-tail and
fuselage pod at full size. This is phase 1 of the flight test plan in
[DESIGN.md](DESIGN.md) §11: the aircraft that gets you a measured sink polar
before you commit to composite moulds.

It is **not** the solar aircraft. The array, the energy-recovery turbine and
the retractable motor pylon all need the moulded airframe — foam has neither
the stiffness to carry a flat cell array without cracking cells nor the
internal volume for the mast trunk. The planform, section, tail volumes and CG
are identical, so what you learn transfers directly.

## Sheets

| Sheets | |
|---|---|
| 1 | Title, print check, key numbers, sheet index |
| 2 | Build notes — tiling, two ways to make the wing, build order, balance |
| 3–9 | Wing panel, full size. Cut 2, one mirrored |
| 10–11 | Aerofoil templates, FF-SC1 at 195 / 160 / 128 / 92 mm chord |
| 12–17 | Fuselage pod, side and plan |
| 18–20 | Formers |
| 21–22 | V-tail panel. Cut 2, one mirrored |

## Tiling

Tiles overlap by 12 mm. Trim along the dashed line on the **overlapping** sheet
only, lay it over its neighbour, line up the solid triangles and tape. Corner
crosses coincide when it is right. Work along a row, then join rows. Each sheet
is labelled `R<row>C<column>`.

## Two ways to make the wing

**Hot-wire cores** — accurate, needs a wire bow. Transfer the aerofoil sheets
to 1.5 mm ply or aluminium, pin them at the right stations on an XPS block and
wire between them. This gives the true FF-SC1 section.

**Flat foam** — quick, no tools. Cut the planform from 6 mm Depron, score the
underside along the spar line and fold a leading-edge wrap over the carbon
tube. Costs about 15 % of L/D against the true section; planform and CG are
unchanged.

## Materials

Quantities are for one aircraft, taken from the plan geometry.

### Foam — which type matters

| Part | Material | Quantity |
|---|---|---|
| Wing cores *(hot-wire route)* | **XPS** insulation board, 25 mm, 30–35 kg/m³ | 2 blocks 1000 × 205 × 24 mm — one 1250 × 600 board does both |
| Wing *(flat-foam route)* | **Depron / XPS sheet, 6 mm** | 0.36 m² planform + LE wrap → one 1250 × 800 sheet |
| Fuselage pod sides | Depron, 3 mm | 2 off, 620 × 96 mm |
| V-tail | Depron or XPS, 5 mm | 2 off, 300 × 140 mm |
| Formers | 3 mm liteply *(or 5 mm Depron)* | 8 off, largest 80 × 96 mm |

**XPS** — extruded polystyrene, the blue/pink/grey building insulation board.
Closed-cell, smooth, hot-wires cleanly, cheap. This is the one for cut cores.

**EPS** — the white beady packaging stuff. Cuts, but the bead texture wrecks
the surface finish and it is weaker for the same density. Not for the wing.

**EPP** — tough and bouncy, survives crashes. Too floppy and too heavy here;
it would blunt the section and spoil the polar measurement, which is the whole
point of this airframe.

### Everything else

| | |
|---|---|
| Main spar | 8 mm OD × 6 mm ID pultruded carbon tube, 2 × 1000 mm, plus a 6 mm joiner ~200 mm |
| Tail boom | 16 mm OD carbon tube, 660 mm (90 mm buried in the pod) |
| Wing skin | 25 g/m² glass cloth + laminating epoxy, **or** heat-shrink laminating film / fibre-reinforced packing tape |
| Adhesive | Epoxy, PVA/white glue, foam-safe CA, or hot glue |
| Hinges | Clear packing tape, or Blenderm surgical tape |

### The mistake that ruins foam builds

**Polystyrene dissolves in petrol-based solvents, polyester resin and ordinary
CA accelerator.** Standard cyanoacrylate and rattle-can paint will eat straight
through XPS and Depron. Use epoxy, PVA, *foam-safe* CA, or hot glue, and test
any paint on an offcut first.

### The skin is not optional

A bare foam wing has roughly twice the parasite drag of a skinned one — the
open cell texture trips the boundary layer everywhere. The C_D0 of 0.031 that
the performance prediction assumes is for a skinned, sanded surface. Skip the
skin and you will measure an L/D nearer 11 than 15.6, and you will wrongly
conclude the design is bad.

## Predicted performance

At the 833 g target mass in the plan's budget, with C_D0 taken as 0.031 for a
foam build (rougher surface, blunter leading edge, exposed linkages):

| | |
|---|---|
| Wing loading | 23.3 g/dm² — lighter than the composite aircraft |
| Stall | 5.6 m/s |
| Best glide | L/D 15.6 at 6.4 m/s |
| Minimum sink | 0.41 m/s |

It flies slower and thermals better than the FF-1 because it carries no array,
no turbine and no mast.

## Then measure it

Fit a flight controller and an **airspeed sensor**, fly the polar sortie in
[FLIGHT_ANALYSIS.md](FLIGHT_ANALYSIS.md) §4, and run:

```bash
python3 tools/flight_analysis.py YOURLOG.BIN --mass 0.83
```

That returns the measured sink polar and the drag level against this
prediction. Closing that loop — print, cut, fly, log, fit — is the entire
point of building the foam one first.

## Regenerating

Every dimension comes from `tools/config.py` through `tools/build_model.py`,
so the plans, the 3D model, the analysis and the mass budget cannot drift
apart. Change a chord there and re-run `python3 tools/plans.py`.
