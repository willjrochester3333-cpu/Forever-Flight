# Optimising the wing, and proving the model matches

```
python3 tools/optimise.py       the search
python3 blender/measure.py      measure the built mesh against the spec
python3 blender/render_dims.py  dimensioned views
```

## What was wrong

The wing had a **94.4 mm tip chord**. At best-glide speed that puts the
outboard sections at **Reynolds 48,700** — below about 55,000 a low-Re
aerofoil's drag rises sharply and it stops reaching the lift coefficient you
designed around. The quoted L/D of 19.6 was a number the aircraft could not
actually have delivered.

Nothing else was badly wrong. Everything the optimiser changed, it changed to
fix that one thing.

## How the search works

Induced drag is most of the drag of a high-aspect-ratio wing at soaring
speeds, and it depends on **how lift is spread across the span** — which is
set by taper and washout. `CL²/(π·AR·e)` with a guessed `e` cannot see that,
so it cannot optimise it; it would just say "more span".

So the span loading is solved by lifting-line theory:

```
y = -(b/2)cos(th)        Gamma(th) = 2bV * sum A_n sin(n th)

sum A_n sin(n th) [ n.mu + sin th ] = mu sin th (alpha - alpha_L0 + twist)

CL = pi.AR.A1      CDi = pi.AR. sum n.A_n^2      e = A1^2 / sum n.A_n^2
```

Span efficiency comes out as an **output of the planform** rather than an
assumption about it, and the same coefficients give the local lift coefficient
at every station — which is what the tip-stall constraint needs.

Validated against known results: `e` = 0.937 for a rectangular wing, 0.979 at
taper 0.45 (the textbook optimum is 0.35–0.45), and washout costs `e` exactly
as it should (−5° drops it to 0.73).

The system matrix depends only on the planform, and the right-hand side is
linear in angle of attack — so it is factored **once** per geometry and every
angle afterwards is a vector combination. That is a 34× speedup and it is the
only reason a three-variable search finishes in a minute.

## What it found

| design | taper | root | tip | washout | AR | area | L/D | sink | Re tip |
|---|---|---|---|---|---|---|---|---|---|
| before | 0.363 | 260 | 94.4 | −2.20 | 11.63 | 36.50 | 19.59 | 0.367 | **48,700 ✗** |
| max L/D, root free | 0.554 | 193 | 106.9 | −2.71 | 13.74 | 30.89 | **20.39** | 0.369 | 55,100 |
| max L/D, root 260 | 0.410 | 260 | 106.6 | −0.34 | 11.24 | 37.76 | 19.47 | 0.373 | 55,700 |
| min sink, root free | 0.476 | 227 | 108.1 | −2.25 | 12.30 | 34.51 | 19.87 | **0.366** | 55,100 |
| **chosen** | **0.408** | **260** | **106.1** | **−1.98** | **11.31** | **37.5** | 19.45 | 0.367 | **55,700** |

**Every optimum lands the tip at 106–108 mm.** That is the Reynolds constraint
binding, in all four searches, whatever the objective. It is the only thing
the wing actually needed.

### Why the chosen one

- **Root stays at 260 mm.** Letting it fall to 193 mm buys +4.8% L/D, but it
  breaks the shared joint with the standard wing. That is a kit-architecture
  decision, not an optimiser's call.
- **Minimum sink, not maximum L/D.** They are nearly the same design here
  (L/D 19.45 vs 19.47), but the max-L/D variant does it with −0.34° of
  washout — almost none. On a swept wing that is thin tip-stall insurance for
  0.1% of glide. −1.98° keeps the margin and gives better sink.
- **Tip loaded to 0.69 of peak**, with the peak well inboard, so it stalls at
  the root first — which on a swept wing is the difference between a recovery
  and a pitch-up.

The headline number went *down*, 19.59 → 19.45. That is honest arithmetic: the
old figure assumed a tip working at a Reynolds number it was not working at.
19.45 is a number the aircraft can have.

## Measuring what was actually built

Every dimension is a number in a dictionary. That is not the same as the model
*having* it — a rotation in the wrong frame, a mirror about the wrong object,
a loft that drops a section, and the model quietly disagrees with its own
specification. Both of those have already happened in this file's history.

So `blender/measure.py` measures the **evaluated geometry, after modifiers, in
world space** and compares:

```
  dimension                    measured  specified     error
  wing span                     2060.00    2060.00     +0.00
  root chord                     260.00     260.00     +0.00
  tip chord                      106.05     106.10     -0.05
  LE sweep                        23.91      24.00     -0.09
  dihedral                         1.86       2.00     -0.14
  wing area (gaps deducted)       37.54      37.49     +0.05
  nozzle exit diameter            40.50      40.50     +0.00
  fan diameter                    48.60      48.60     -0.00

  aspect ratio  11.31    taper 0.408
  built mass 1121 g -> wing loading 29.9 g/dm2   (published envelope 28-48)

  ALL DIMENSIONS WITHIN TOLERANCE
```

It exits non-zero if anything is out, so it works as a check rather than a
report.

Three things it caught while being written, all of them errors in the
*measuring* rather than the model — which is the point of measuring from the
mesh instead of trusting a formula:

- **Span read 2152.7 mm.** Correct — that is over the winglets, which cant
  outboard past the tip. Wing span and tip-to-tip are now separate rows.
- **Area read 6.7 dm².** Integrating over fixed spanwise bands finds nothing,
  because a lofted mesh only has vertices where a section was placed. Areas
  now come from projecting every polygon.
- **Area then read 29.6 dm².** The flaps and ailerons are separate objects,
  cut out of the wing so they can hinge, so "Wing" alone is missing a quarter
  of the chord over three quarters of the span.

The remaining 0.4% is the **hinge gaps** — eight 3 mm gaps at the ends of four
control surfaces. Rather than widen the tolerance to hide it, the specified
area has the gap deduction computed into it, so the comparison stays exact.

## Dimensions on the model

`blender/render_dims.py` writes a `Dimensions` collection — leader lines with
end ticks and text labels for span, root chord, tip chord, overall length,
area, aspect ratio and sweep. The numbers are read off the **measured**
geometry, not off the parameters, so if the two ever disagree the drawing
shows the truth.
