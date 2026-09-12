# Flight log analysis

Feed it a log, get graphs and predictions.

```bash
python3 tools/flight_analysis.py data/example_flight.csv --open
python3 tools/flight_analysis.py 00000042.BIN --mass 1.68
```

Writes a self-contained HTML report to `reports/`. No dependencies — standard
library Python 3.8+, so it runs on the laptop at the flying field without a
`pip install`.

Try it now: `data/example_flight.csv` is a synthetic 2.6-hour flight generated
by `tools/simulate_flight.py` against the real design model, so you can see
the whole pipeline before the aircraft exists.

---

## 1. What it does

| Stage | |
|---|---|
| **Read** | ArduPilot `.BIN` straight off the SD card, ArduPilot `.log` text, or any CSV. Column names are matched fuzzily — whatever your log calls airspeed comes out as `airspeed` |
| **Derive** | climb rate (least-squares slope over a ±2 s window, so baro noise is averaged rather than differentiated), air density, electrical power, cumulative energy |
| **Segment** | powered / thermalling / turbine-deployed / gliding, median-filtered so one banked sample is not a thermal |
| **Fit** | the drag polar, from stabilised glide legs |
| **Predict** | drag level vs design, turbine harvest for that day's thermals, and endurance re-run on the measured drag |

---

## 2. How the polar fit works

Sink rate against airspeed, for a drag polar `C_D = A + D·C_L + B·C_L²`:

```
w(V) = (A/K)·V³ + D·V + (B·K)/V        K = 2W/(ρS)
```

Three terms, linear in the coefficients, so ordinary least squares recovers
them from a handful of stabilised glides. Everything downstream — L/D max,
best-glide speed, minimum sink — then follows from the curve.

Three things the tool does that matter:

**It reduces every leg to sea-level density.** At the same indicated airspeed
you fly faster and sink faster at altitude, both by √(ρ₀/ρ), so each leg's
sink is corrected before fitting. Otherwise a polar flown from 500 m down to
200 m has a built-in trend.

**It drops legs that were flown in lift.** A leg through weak rising air reads
as impossibly low sink and drags the whole fit with it. Points more than 2σ
*below* the curve are rejected and refitted (two passes); they appear as
hollow circles on the polar. Points above the curve are kept — sink and
turbulence are honest.

**It picks the fit order by cross-validation.** Two terms (the textbook
`aV³ + b/V`) forces `D = 0`, which biases both C_D0 and e — on synthetic data
with a known answer, by 12 % and 20 %. Three terms fits better but needs more
points. Leave-one-out CV decides, and the report prints both errors.

---

## 3. Read the confidence flags, not just the numbers

A sink curve determines **total** drag well. Splitting it into parasite and
induced drag is a different question, and often the data cannot answer it: over
a narrow speed range V³, V and 1/V are nearly collinear, so very different
coefficient pairs fit the same curve almost equally well.

So the report bootstraps — resamples the glide legs 300 times, refits, and
quotes 10–90 % intervals. On the example flight:

| | Fitted | Interval | Verdict |
|---|---|---|---|
| L/D max | 18.07 | 17.8 – 18.4 | usable, ±2 % |
| Speed for best glide | 10.04 m/s | 9.8 – 10.1 | usable, ±2 % |
| C_D0 | −0.048 | −0.088 – −0.022 | **not identifiable** — a negative C_D0 is impossible |

That is the tool working correctly. The curve is nailed; the decomposition is
not, and it says so rather than printing a confident wrong number.

**So the headline drag metric is a single scale factor:** the multiplier on
design C_D0 that reproduces the measured L/D. One parameter fitted to one
well-determined quantity, always identifiable, and it is what you actually
want to know — *is the aircraft draggier than predicted, and by how much?*
On the example it comes back ×0.95 [0.92–0.98], against a true value of ×1.00.

---

## 4. Fly a polar sortie properly

This is phase 1 of the flight test plan in [DESIGN.md](DESIGN.md) §11, and it
is the single most valuable data set in the programme.

- **Calm air.** Early morning or late evening. Thermals are the dominant error
  source and no amount of fitting fixes them.
- **Seven speeds or more**, from just above stall to about 2.5× stall — for the
  FF-1 that is roughly 8, 9, 10.5, 12, 14, 17, 20 m/s. A narrow range is what
  makes C_D0 unidentifiable.
- **Three repeats per speed**, non-consecutive, so a single gust does not
  become a data point.
- **30 s per leg**, wings level within ±5°, motor off, airspeed steady to
  ±0.3 m/s. The tool requires 20 s windows with bank under 6° and airspeed
  acceleration under 0.05 m/s²; anything looser is rejected.
- **Know the mass.** Pass `--mass` in kg. The polar scales with √(W/S) and the
  coefficients scale with W directly.
- **Log airspeed.** Groundspeed is a fallback and the report warns loudly when
  it has been used: a 3 m/s wind biases L/D by tens of percent depending on
  heading.

---

## 5. What to log

Channels the tool understands. Everything is optional except `alt`, and
`airspeed` for the polar.

| Channel | Unit | Why |
|---|---|---|
| `alt` | m | required; climb rate comes from it |
| `airspeed` | m/s | required for the polar |
| `roll` | deg | separates thermalling from gliding, rejects banked legs |
| `throttle` | 0–1 | separates powered flight from gliding |
| `volt`, `curr` | V, A | pack power and energy |
| `solar_w` | W | MPPT output — **FF-1 specific, add this message** |
| `turbine_w` | W | turbine controller output — **FF-1 specific** |
| `mast_mm` | mm | turbine mast extension — **FF-1 specific** |
| `temp` | °C | outside air temperature, improves the density correction |
| `groundspeed`, `pitch`, `used_mah`, `lat`, `lon` | | used where present |

`python3 tools/flightlog.py` with no arguments prints the full alias list.

The three FF-1-specific channels need a custom ArduPilot log message from the
power subsystem. They are worth the trouble: without them the energy
accounting is a single battery number, and the turbine's contribution — the
thing the whole §5.5 argument in [ANALYSIS.md](ANALYSIS.md) rests on — cannot
be measured at all.

---

## 6. Options

| | |
|---|---|
| `--mass KG` | flight mass; defaults to design MTOW |
| `--dt S` | resample interval, default 0.5 s |
| `--window S` | glide window length for polar legs, default 20 s |
| `-o PATH` | output file |
| `--open` | open the report in a browser when done |

## 7. Charts

Colours come from a categorical palette validated for colour-vision
deficiency, contrast and chroma in both light and dark mode. Fixed assignment,
never cycled: **orange = powered/motor, teal = turbine/regen, amber = solar,
purple = measured/glide**. Status green and red are reserved for the
confidence flags and never used as series colours.

The time-series charts have a crosshair and tooltip on hover. The report is
one self-contained file — no network needed except web fonts, which degrade to
system fonts offline.
