# JW-1 — a swept flying wing in the Jetwing configuration

```bash
python3 tools/jetwing.py
```

Writes `models/jetwing/jetwing-1500.stl` / `.obj` (the larger wing) and
`jetwing-1200` (standard), plus renders.

## What this is, and what it is not

**It is not a copy of PlanePrint's Jetwing.** `www.planeprint.com` is blocked
by this session's network egress policy — the proxy returns 403 on CONNECT —
so none of their published dimensions, weights, motor sizes or CG figures were
available, and a single photograph does not give geometry. Reproducing a
commercial model's STL would not be the right thing to do even with access.

What this **is**: a swept flying wing designed from scratch to the
configuration in the photo — swept tapered wing, upturned winglets, single
dorsal fin, pusher prop on a blended centre pod — with the aerodynamics
actually worked rather than eyeballed. Every dimension is a parameter in
`JW` at the top of `tools/jetwing.py`. Give me PlanePrint's real numbers and
they drop straight in.

## The part that makes a tailless wing different

There is no tailplane to balance the wing's nose-down pitching moment, so the
wing has to balance itself. Two mechanisms, and both are tuned here against
theory rather than guessed:

**Reflex** — the trailing edge curves up, which flips the section's own moment
from nose-down to nose-up. The reflex is solved by bisection against
thin-aerofoil theory to hit a target Cm:

```
A1 = (2/π) ∫ dz/dx · cos θ dθ        Cm(c/4) = (π/4)(A2 − A1)
A2 = (2/π) ∫ dz/dx · cos 2θ dθ       α_L0   = −(1/π) ∫ dz/dx (cos θ − 1) dθ
```

The implementation is checked against NACA 4412, where it returns
α_L0 = −4.15° against a published −4.1°.

**Washout** — the tips are twisted nose-down, and on a *swept* wing the tips
sit behind the CG, so they work as a tailplane. Trim is solved by strip
theory over 80 spanwise strips: the CG is found by bisecting for the point
where dCm/dCL = 0 (the neutral point), then set forward of it by the static
margin, and the trim angle found by bisecting for Cm = 0.

A finding from the sweep that surprised me: **camber barely moves the trim
point.** More camber needs more reflex to hold the same Cm0, and the two
cancel in α_L0. Washout and the Cm0 target are the real levers.

## Specification — larger wing

| | |
|---|---|
| Span | 1500 mm (1583 mm tip to tip over the winglets) |
| Length / height | 621 / 281 mm |
| Wing area | 3675 cm², AR 6.12, taper 0.44 |
| LE sweep | 24° |
| MAC | 257.3 mm at y = 327 mm |
| Section | 10.5 % thick, 2.2 % camber, reflex 0.0196 aft of 65 % chord |
| | → Cm(c/4) **+0.0060**, zero-lift angle +0.87° |
| Washout | −3.0° at the tip |
| Neutral point | 25.0 % MAC |
| **Centre of gravity** | **263 mm from the nose = 17.5 % MAC** |
| Static margin | 7.5 % |
| Trims at | 6.5° for C_L 0.366 |
| At 880 g | 23.9 g/dm², trim 10.2 m/s, stall 6.7 m/s |

The 1200 mm version is the same shape scaled: CG 211 mm from the nose, 493 g,
stall 6.3 m/s.

## Why 3° of washout and not less

A swept wing that stalls at the tip first pitches **nose-up**, because the
tips are behind the CG. On an aircraft with a tailplane that is recoverable.
On a tailless wing it is a departure — the stall deepens itself. Three degrees
is the usual insurance and it costs very little: the trim sweep shows it moves
trim C_L from 0.16 (no washout) to 0.37.

## Build notes

- **Set the CG before anything else.** 17.5 % MAC with a 7.5 % static margin is
  a sport setting. If it feels pitchy, move the battery forward; a tailless
  wing gives much less warning than a conventional one.
- Elevons are scribed at 28 % chord from 34 % to 96 % semi-span.
- The pod datum is set 5° nose-down against the root chord so the body looks
  level at the 6.5° trim attitude.
- The fin is sized by eye from the photo, not by a yaw-stability calculation —
  a flying wing's directional stability depends on winglet area as much as the
  fin, and I would want the real aircraft's numbers before claiming one.

## Changing anything

Everything is in the `JW` dict:

```python
"span": 1500.0, "root_chord": 340.0, "sweep_le": 24.0,
"washout_tip": -3.0, "target_cm": 0.006, "static_margin": 0.075,
```

Change a value and the geometry, trim solution, CG and spec sheet all follow.

## Thrust-vectoring tail

The JW-1 can be built with a cruciform tail in the propeller slipstream instead
of the dorsal fin — see [VECTOR_TAIL.md](VECTOR_TAIL.md). It gives 1.9× the
elevon pitch authority at hand-launch speed, adds the rudder the JW-1 never had,
guards the pusher propeller on landing, and lowers the trim speed from 10.2 to
8.4 m/s. It costs 54 g, 2.5 % of best-glide sink, and a nose-bay rebuild to move
the pack 72 mm forward.

Set `cfg["vt"] = True`, or build the `jetwing-1500-vt` / `jetwing-1200-vt`
variants. The analysis found that the dorsal fin it replaces was earning
V_v = 0.0005 — effectively nothing.
