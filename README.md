# Forever-Flight — FF-1 *Ember*

A 2 m span autonomous soaring glider for wildfire detection, with a
retractable motor pylon and a retractable energy-recovery turbine.

![FF-1 Ember](docs/img/iso_deployed.png)

| | |
|---|---|
| Span | 2.00 m |
| MTOW | 1622 g |
| Best glide | L/D 17.6 at 9.5 m/s |
| Minimum sink | 0.51 m/s |
| **Endurance** | **10.0 h** (solar + thermal soaring, 38° N, August) |
| Endurance, battery only | 1.4 h |
| Survey coverage | 10.1 km²/h → 81 km² per sortie |
| Detection floor | ~0.5–1 m² of active flame, clutter-limited |

## What the dynamo actually does

It works. In steady level flight a turbine cannot pay for itself — extracting
power costs more power, and that shows up as drag. But this aircraft doesn't
fly steady level flight. **It soars: it climbs on atmospheric energy and
descends on the turbine, so the source is the air, not the pack. Over that
cycle the output is unambiguously net positive.**

The turbine takes whatever the atmosphere offers beyond what the airframe
needs to stay up. On a good day that's **3.6 W continuous —
46 % of the 7.9 W hotel load — and about
14 Wh over a flight**, a quarter of the pack.

**The catch is storage, not generation.** On a clear day the array has the
pack full by mid-morning, so all of it is spilled. The turbine pays exactly
when the array can't: with one solar string failed it's worth +0.6 h, in heavy
overcast +0.9 h, and with a badly degraded array +4.6 h. That's the argument
for any redundant system — it costs nothing on the days it isn't needed,
because it's stowed.

The deploy rule follows: **only harvest when the pack has room.** On a full
pack the surplus should go into altitude or survey coverage instead.

Full working in [docs/ANALYSIS.md](docs/ANALYSIS.md) §5.5.

## Documents

| | |
|---|---|
| [docs/DESIGN.md](docs/DESIGN.md) | The design plan — configuration, aerodynamics, structure, energy, mechanisms, autonomy, payload, flight test, risks |
| [docs/ANALYSIS.md](docs/ANALYSIS.md) | Every computed number, generated from source |
| [docs/VERTICAL_MAST.md](docs/VERTICAL_MAST.md) | VDM-1 — the telescoping vertical deployment mast and its micro linear actuator |
| [docs/BUILD_MANUAL.md](docs/BUILD_MANUAL.md) | Bill of materials and construction sequence |

## The 3D model

```
models/
  ff1_deployed.stl     both pods extended
  ff1_stowed.stl       both pods retracted
  ff1_deployed.obj     named groups, for CAD import
  ff1_stowed.obj
  geometry.json        parametric data, including the solar strip layout
  parts/*.stl          each component separately
```

The assembly STLs are **visual/assembly models**: components interpenetrate at
their joins, so the union is not a closed manifold. Boolean the parts in
Blender or Meshmixer before slicing, or print from `parts/` individually. The
wing meshes on their own are watertight.

Units are millimetres; +X aft with the nose at the origin, +Y starboard, +Z up.

## Regenerating

```bash
python3 tools/build_model.py   # models/
python3 tools/analysis.py      # docs/ANALYSIS.md
python3 tools/render.py        # docs/img/
```

No dependencies — standard library only, Python 3.8+.

`tools/config.py` is the single source of truth. Change a dimension there and
the model, the drawings, the mass budget, the drag polar, the energy budget
and the detection performance all regenerate consistently.

| | |
|---|---|
| `tools/config.py` | Every dimension, mass, efficiency and mission parameter |
| `tools/airfoil.py` | NACA 4-digit generator plus the curvature-limiting "solar flattening" morph |
| `tools/mesh.py` | Triangle mesh kernel, STL and OBJ writers |
| `tools/build_model.py` | Lofted airframe, deployable pods, solar cell layout |
| `tools/analysis.py` | Drag build-up, polar, turbine physics, clear-sky solar model, mission simulation, stability, wing beam, LWIR detection |
| `tools/mast.py` | VDM-1 sizing: stroke, stage count, drive loads, actuator selection, resonance |
| `tools/render.py` | Z-buffered PNG renderer for the three-views |

## Status

Design study with a complete parametric model and a closed analysis. Nothing
has been built or flown. The aerofoil is a geometric mould definition, not a
validated section — see [docs/DESIGN.md](docs/DESIGN.md) §2.1 before cutting
anything.

Flying near an active wildfire without coordination with the incident
commander is dangerous and illegal in most jurisdictions — it grounds the air
tankers. See [docs/DESIGN.md](docs/DESIGN.md) §13.
