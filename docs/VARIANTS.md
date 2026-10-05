# Airframe variants

```bash
python3 tools/variants.py
```

Writes four 3D models to `models/variants/` as STL and OBJ, plus renders to
`docs/img/variants/`.

These are the aircraft **as the printed plans actually produce it**: no
retractable motor pylon, no retractable turbine mast, and therefore no dorsal
spine and no keel fairing either — those two exist only to swallow the
retracting pods, so with the pods gone they are pure drag and they go too.

| Variant | Span | Motor | Mass | Wing loading | Stall | L/D | Min sink |
|---|---|---|---|---|---|---|---|
| `glider-2000` | 2.00 m | none | 741 g | 20.7 g/dm² | 5.3 m/s | 16.7 | 0.36 m/s |
| `motor-2000` | 2.00 m | nose folder | 833 g | 23.3 g/dm² | 5.6 m/s | 16.6 | 0.39 m/s |
| `glider-2500` | 2.54 m | none | 1059 g | 18.9 g/dm² | 5.0 m/s | 16.7 | 0.35 m/s |
| `motor-2500` | 2.54 m | nose folder | 1151 g | 20.6 g/dm² | 5.3 m/s | 16.6 | 0.37 m/s |

C_D0 is 0.0316 (0.0320 with the nose motor), built from the design component
table with the spine, keel fairing, bay doors and solar-cell steps removed and
a foam-surface penalty added. The composite FF-1 is 0.0247.

## The motor

Nose-mounted folding prop, which is available because the sensor turret is
slung **under** the nose rather than on it. No pylon, no bay, no actuator — the
blades simply fold back against the fuselage when the motor is off. This is
"Variant N" from [DESIGN.md](DESIGN.md) §10, which already came out lighter and
lower-drag than the retractable pylon.

## The larger version is scaled whole, not re-winged

`glider-2500` and `motor-2500` are the entire aircraft at ×1.25 — wing, tail,
fuselage, everything. That is deliberate.

Geometric scaling preserves the ratios that keep an aircraft trimmed and
stable. Tail volume coefficients and CG position come out **identical**:

| | V_h | V_v | CG |
|---|---|---|---|
| Both spans, scaled whole | 0.551 | 0.0306 | 34.2 % MAC |
| Span stretched to 2.5 m on the same fuselage and tail | 0.441 | 0.0196 | — |
| Target band | 0.45 – 0.70 | 0.020 – 0.035 | — |

Bolting a longer wing onto the existing fuselage drops **both** tail volumes
just below their bands — the fin loses a third of its authority. It would fly,
but it would wander in yaw and want a bigger tail to fix. Scaling everything
avoids the problem instead of trading against it.

One free bonus: the 2.5 m version has *lower* wing loading than the 2.0 m
(18.9 against 20.7 g/dm²), because the electronics and battery do not scale
with the airframe. It will thermal better.

## Mass scaling

Structure scales with the geometric factor, bought gear does not:

| Scaling | Applies to |
|---|---|
| k³ (volume) | wing cores, carbon spar |
| k² (area) | skins, pod, boom, tail, control surfaces |
| k⁰ (fixed) | motor, ESC, battery, RX, servos, flight controller |

## Changing the span

Edit `VARIANTS` in `tools/variants.py` — the scale factor is the second field:

```python
("glider-3000", 1.50, "none", "3.0 m glider"),
```

Everything downstream follows: geometry, mass, performance, stability.

## Files

```
models/variants/glider-2000.stl   .obj
models/variants/motor-2000.stl    .obj
models/variants/glider-2500.stl   .obj
models/variants/motor-2500.stl    .obj
docs/img/variants/*.png           renders, plus size-comparison.png
```

The OBJ files carry named groups (`wing_stbd`, `fuselage`, `vtail_port`,
`nose_motor`, …) so you can hide or re-colour parts in a CAD or 3D tool. The
STL is a visual/assembly model: components interpenetrate where they join, so
boolean them before slicing if you intend to print one.

Units are millimetres; +X aft with the nose at the origin, +Y starboard, +Z up.
