# The 50 mm EDF installation

Fixed. Nothing retracts, nothing folds, nothing protrudes. The whole shape is
arranged to make that cost as small as it can be.

![EDF](img/blender/jetwing-edf.png)

Run the sizing: `python3 tools/edf.py`
Build the model: `blender --python blender/jetwing.py`

## The unit

| | |
|---|---|
| fan | 50 mm, 12 blades, QF2611 5000KV, 9N6P |
| mass | 109 g complete (motor alone 76 g) |
| rating | 550 W / 45 A maximum continuous |
| quoted | 950 g static at 12.6 V, 45 A, 567 W |

12.6 V is a **full 3S pack, not 4S**. On 4S the same 45 A would be 666 W, past
the 550 W rating — so on 4S this unit is *power* limited, not voltage limited.
4S buys lower current, cooler running and thinner wiring, not much more thrust.

My duct model gives 915 g static against their 950 g claim on the same power,
and 806 g on a realistic fan efficiency. Call it **800 g**.

## The number that governs everything

At 16 m/s the fan throws a **72 m/s jet**. That is 4.5× flight speed, and
propulsive efficiency is what is left over:

```
eta_Froude = 2V / (V + Ve) = 2(16) / (16 + 72) = 36.4%
```

With fan and motor losses on top, **22.6%**. That is not a defect in the
installation — it is what a 50 mm fan on a slow aircraft *is*. A 70 mm fan,
which is what the kit calls for, moves 2× the air at a much lower jet speed
for the same thrust, and that is where its efficiency comes from.

Everything below is about not making it worse.

## Design choices, and why

**Nozzle at 0.90 × fan swept area → 40.5 mm.** Exit area is the biggest lever
there is. Opening it gives a slower, fatter jet: more thrust *and* better
efficiency.

| Ae/FSA | dia | thrust | jet | η_p |
|---|---|---|---|---|
| 0.80 | 38.2 | 628 g | 74.7 | 21.9% |
| **0.90** | **40.5** | **648 g** | **72.0** | **22.6%** |
| 1.00 | 42.7 | 667 g | 69.6 | 23.2% |

It looks like you should open it all the way. You cannot: past about 1.00 the
fan unloads, overspeeds and stalls its own blades. 0.90 is the standard pick
and it is what is built.

**Bellmouth inlets, 3.1 mm lip radius.** The duct swallows 114 g/s. At 16 m/s
that is a free-stream tube 3.9× the inlet area, so flow accelerates *in* and
nothing spills — spillage would only start above 62 m/s, which this aircraft
never sees. But inlet velocity is 62 m/s, so the lip has to turn nearly still
air very hard. A sharp lip separates right there, and that is where cheap
installations quietly lose 10–20% of their thrust. The lip is rounded to 10%
of inlet diameter.

**Diffuser half-angle 1.2°, nozzle half-angle 3.0°.** Above about 7° a
diffuser separates and you lose the pressure recovery the duct exists for.
1.2° is very safe. Converging sections are always safe.

**Seven stator vanes against twelve rotor blades.** The rotor leaves swirl in
the jet — rotational energy you have already paid for and get no thrust from.
The stator straightens it out. Seven is prime against twelve so blade passing
never lines up into a tone.

**A closing tailcone behind the motor.** A motor can that just stops leaves a
separated base, which is pure drag inside your own duct.

## What it costs when the fan is off

This is the number that decides a fixed installation on a soaring aircraft,
because gliding is most of the flight:

| | freewheeling | braked |
|---|---|---|
| 10 m/s | 0.033 N | 0.046 N |
| 14 m/s | 0.064 N | 0.090 N |
| 18 m/s | 0.106 N | 0.149 N |

At 14 m/s the aircraft's total drag is roughly 0.6 N, so a freewheeling duct
is about **10% of it** and a braked one about 15%.

**Set the ESC to coast, not brake.** A stopped fan is a flat plate across the
duct; a freewheeling one still passes air. It is a menu setting and it is
worth a third of the duct's cold drag.

## Full output

```
== 50 mm EDF installation, designed for cruise at 16 m/s ==

As drawn: 648 g thrust, jet 72.0 m/s, propulsive efficiency 22.6%
  ideal (Froude) ceiling at that jet speed is 36.4% -- the fan is small, so the jet
  overshoots flight speed 4.5x and most of the power goes into the air, not the aircraft.

NOZZLE AREA, the biggest lever
  Ae/FSA   dia    thrust    jet    eta_p   Froude
   0.80    38.2    628 g   74.7   21.9%   35.3%
   0.85    39.4    639 g   73.3   22.2%   35.8%
   0.90    40.5    648 g   72.0   22.6%   36.4%
   0.95    41.6    658 g   70.7   22.9%   36.9%
   1.00    42.7    667 g   69.6   23.2%   37.4%
   1.05    43.8    675 g   68.5   23.5%   37.9%
  Opening the nozzle trades static thrust for efficiency. 0.90 is
  the usual compromise and is what is built; going past 1.00 unloads
  the fan into blade stall, so it is not free.

INLET
  swallows 114 g/s; at 16 m/s that is a free-stream tube 3.9x the inlet area,
  so the flow accelerates IN and nothing spills. Spillage would only
  start above 62 m/s, which this aircraft never sees.
  inlet velocity 62 m/s -> the lip has to turn still air
  hard, so round it to 3.1 mm. A sharp lip separates and
  that is where cheap installations lose 10-20% of their thrust.

INTERNAL ANGLES
  two 30.9 mm inlets -> 43.8 mm equivalent -> 50 mm fan
  diffuser half-angle 1.2 deg (keep under 7, or it separates)
  nozzle half-angle 3.0 deg, converging, which is always safe

WHAT IT COSTS WHEN THE FAN IS OFF
  10.0 m/s: freewheeling 0.033 N, braked 0.046 N
  14.0 m/s: freewheeling 0.064 N, braked 0.090 N
  18.0 m/s: freewheeling 0.106 N, braked 0.149 N
  A soaring aircraft glides far more than it motors, so this is the
  number that decides the installation. LET THE FAN FREEWHEEL --
  set the ESC to coast, not brake. A stopped fan is a flat plate.
```
