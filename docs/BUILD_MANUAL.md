# FF-1 *Ember* — build manual

Companion to [DESIGN.md](DESIGN.md). Dimensions come from `tools/config.py`;
regenerate `models/` after any change.

Part numbers below are **examples that meet the requirement**, not
endorsements, and availability moves — treat the requirement column as the
specification and substitute freely.

---

## 1. Bill of materials

### 1.1 Structure

| Qty | Item | Requirement | Example |
|---|---|---|---|
| 1 m² | Biaxial carbon cloth, 80 g/m² ±45° | D-box torsion skin | — |
| 0.5 m² | UD carbon, 115 g/m² | Spar caps, 9 mm tows | T700-based |
| 0.5 m² | Glass cloth, 25 g/m² | Aft skin, shear web | — |
| 1 | XPS or Rohacell block, 60 kg/m³ | Wing cores, hot-wire cut | 1100 × 250 × 60 mm |
| 1 | Carbon boom, tapered 22 → 16 mm | Tail boom, 700 mm | — |
| 1 | Carbon tube, 12 mm × 250 mm | Wing joiner | Pultruded, not rolled |
| 2 | Aluminium joiner sleeve, 12 mm ID | Centre-panel sockets | — |
| 1 set | Epoxy laminating resin, slow hardener | 40–60 min pot life | — |
| 1 | Moulding board / plug stock | Fuselage plug | Tooling foam or MDF |

### 1.2 Energy

| Qty | Item | Requirement | Note |
|---|---|---|---|
| 12 | Monocrystalline cells, 125 mm, ≥ 22 % | Cut into thirds → 36 strips; 34 used | Buy 12 for 11.33 — cutting yield is not 100 % |
| 1 | Tabbing ribbon + flux pen | Low-temperature, thin | Thick ribbon costs drag |
| 6 | Schottky bypass diodes | One per 7 strips | Surface-mount, under the skin |
| 1 m² | ETFE encapsulation film, 50 µm | Optically clear, UV stable | Outermost layer |
| 1 | MPPT, 3 independent channels | 8–14 V in, 12.6 V out, ≥ 40 W total | 3 channels is not optional — see DESIGN §4.1 |
| 3 | Li-ion 21700, ≥ 4.9 Ah, **≥ 10 A continuous** | Series 3S1P | Capacity cells with 7 A ratings will not do |
| 1 | 3S BMS with balance | ≥ 15 A, low quiescent | |
| 1 | Fuse, 15 A, and an XT30 main disconnect | | |

### 1.3 Propulsion and recovery

| Qty | Item | Requirement | Note |
|---|---|---|---|
| 1 | Brushless outrunner, ~500 Kv, ≤ 55 g | Motor, 3S | 480–540 Kv acceptable |
| 1 | ESC, 30 A, **bidirectional / regen capable** | Motor drive | |
| 1 | Folding prop 10 × 6 + spinner + hub | 254 mm | Carbon blades |
| 1 | Brushless outrunner, ~380 Kv, ≤ 50 g | Turbine generator | 350–420 Kv; a rewound gimbal motor works |
| 1 | FOC controller with regen | Turbine → 12 V bus | VESC-class mini |
| 1 | Turbine rotor, 150 mm, 4 folding blades | Custom; see §4 | |
| 2 | Metal-gear micro servo, ≥ 3.5 kg·cm, ≤ 14 g | Pylon and turbine actuation | |
| 1 set | 4-bar linkage hardware | 2 mm carbon links, M2 clevises, over-centre stop | |

### 1.4 Avionics and payload

| Qty | Item | Requirement |
|---|---|---|
| 1 | Flight controller, ArduPlane-compatible wing board | F405 or H743 class, integrated baro |
| 1 | Airspeed sensor + pitot | **Essential** — the soaring controller needs energy rate |
| 1 | GNSS + magnetometer | SBAS capable |
| 1 | LoRa telemetry modem, 868/915 MHz | BVLOS command and hotspot reporting |
| 1 | Control link RX | Test phase; keep it fitted |
| 4 | Wing/tail servo, ≤ 9 g, metal gear | 2 aileron, 2 ruddervator |
| 2 | Wing servo, ≤ 9 g | Flaps (optional, recommended) |
| 1 | LWIR radiometric core, 160 × 120, 8–14 µm | Radiometric output is required, not just imagery |
| 1 | Visible camera module | Smoke cross-check |
| 1 | Low-power SBC | Event-triggered inference |

---

## 2. Tooling

Hot-wire cutter with the FF-SC1 templates (generate with `tools/build_model.py`
and cut from 1.5 mm ply or aluminium). Vacuum bagging setup — a 30 kPa pump is
enough and gentler on the cells than a strong one. Digital scale reading to
0.1 g; you will use it constantly. A 1.5 m flat reference table.

---

## 3. Construction sequence

Weigh every component before and after installation and record it against the
mass table in [ANALYSIS.md](ANALYSIS.md) §1. A build that tracks its budget
lands at 1518 g; one that does not lands at 1800 g and loses two hours of
endurance.

### Stage 1 — Wing cores and spar

1. Hot-wire the five core sections (centre pair, inner pair, outer pair) from
   the templates. Mind the washout: −1.4° at y = 850, −2.0° at the tip.
2. Route the spar trench at 30 % chord, full depth, through all cores.
3. Lay up the spar caps directly in the trench: 3 plies at the root, dropping
   to 2 at 45 % semi-span and 1 at 70 %. **Step the plies, do not sand a
   taper** — sanded UD loses its fibre continuity exactly where you need it.
4. Bond the vertical shear web (45/45 glass) between the caps.
5. Cure under vacuum on the flat table with the cores in their cradles.

### Stage 2 — D-box and skin

6. Lay the ±45° biaxial carbon over the leading edge, wrapping to the spar top
   and bottom. This carries the torsion.
7. Skin aft of the spar with 25 g/m² glass.
8. **Rout the cell recesses** — 0.4 mm deep, matching the strip layout in
   `models/geometry.json`. Recessing the cells rather than laying them proud
   is worth roughly half the 0.0012 cell-step drag increment.
9. Wet-sand to 600 grit, then 1200. At Re ≈ 110 000 this matters more than
   almost anything else you can do.

### Stage 3 — Array

10. Cut cells into thirds with a laser or a diamond scribe. Expect breakage;
    that is why the BOM says 12 cells for 11.33.
11. Tab strips in **three independent strings**, series within a string.
    Fit bypass diodes every 7 strips.
12. Dry-fit the whole array before bonding anything. Check every strip sits
    flat in its recess with no rock.
13. Bond with a thin, flexible optically-clear adhesive. Do not use a rigid
    epoxy — it transfers skin strain straight into the silicon.
14. Encapsulate with ETFE, vacuum-bagged at low pressure. Seal the edges.
15. **Electroluminescence-image the array** before the wing ever flies, and
    keep the image. It is your only way to see a crack.

### Stage 4 — Fuselage

16. Plug, mould, then lay up the pod in Kevlar/carbon. Kevlar on the outer
    plies of the belly — it survives belly landings that carbon does not.
17. Bond the spar box at x = 250–300 mm, square to the fuselage datum and
    with 1.5° of wing incidence built in.
18. Cut the dorsal bay (x = 385–790) and the ventral bay (x = 350–575).
    Reinforce both openings with a carbon rim; you have just cut the two
    highest-stress regions of the shell.
19. Bond the boom, checking the V-tail incidence at −1.0° against the wing.

### Stage 5 — Mechanisms

20. Build the pylon 4-bar as a sub-assembly on the bench. Set the over-centre
    stop so the linkage locks 2° past top-dead-centre at full deployment.
21. Cycle it 200 times by hand before installation. Anything that will bind,
    binds in the first 200 cycles.
22. Build the turbine swing arm. Proof-load the folding blade hinges to **3×
    the calculated centrifugal load at 7000 rpm** before you ever spin it.
23. Balance the rotor statically, then dynamically. A 150 mm rotor at 6400 rpm
    is a serious object.
24. Fit bay doors, cammed off each linkage. Verify the doors cannot be open
    with the pod stowed.

### Stage 6 — Systems

25. Wire the 12 V bus as a star from the BMS, not a daisy chain.
26. Route the three MPPT string pairs through the wing joiner as twisted
    pairs. Solar strings are a noise source next to the magnetometer — keep
    them 100 mm clear of it and twist them.
27. Mount the battery on the sliding tray. Do not glue it down; it is the CG
    trim mechanism.
28. **Meter every rail on the bench** and compare to the 7.9 W hotel budget in
    [ANALYSIS.md](ANALYSIS.md) §5.3 before the first flight.

---

## 4. Turbine rotor

The one part with no off-the-shelf equivalent.

| | |
|---|---|
| Diameter | 150 mm |
| Blades | 4, folding |
| Hub radius | 15 mm |
| Chord | 22 mm at the root, 15 mm at mid, 9 mm at the tip |
| Twist | 36° at r/R = 0.2 falling to about 11° at the tip |
| Section | Thin cambered plate, 9 % thick, 3.5 % camber |
| Design point | λ = 2.5, C_p 0.36 |

Blade geometry is generated by `build_turbine()` in `tools/build_model.py` and
exported to `models/parts/rat_pod_deployed.stl`. Print the blades in a tough
filament for ground testing; make the flight set from carbon-filled nylon or
lay them up in carbon over a printed core.

The high twist is deliberate. A turbine blade at low tip-speed ratio sees a
very different inflow angle at the root than at the tip, and an untwisted
blade will be stalled inboard and windmilling outboard.

---

## 5. Setup and first-flight configuration

### Balance
CG at **277 mm aft of the nose** (34.2 % MAC, 20 % static margin). Set it with
both pods **stowed**, wing level, on the spar-box datum. Slide the battery
tray; do not add ballast until the tray is at an end stop.

### Control throws (starting points)
| Surface | Throw | Expo |
|---|---|---|
| Aileron | ±14 mm, 35 % differential | 30 % |
| Ruddervator (pitch) | ±9 mm | 35 % |
| Ruddervator (yaw) | ±12 mm | 25 % |
| Flap | 0 / −6 / −14 mm (cruise / thermal / landing) | — |

### ArduPlane
Set the airspeed sensor up first and confirm it against a measured glide.
Then `SOAR_ENABLE = 1`, `SOAR_VSPEED = 0.7`, `SOAR_ALT_MIN = 250`,
`SOAR_ALT_MAX = 900`, and enter the sink polar from
[ANALYSIS.md](ANALYSIS.md) §2.3 into the `SOAR_POLAR_*` parameters.

### Interlocks to verify on the bench, every time
1. Turbine retracts and disengages below 25 m AGL — **test the hardware
   failsafe with the scripting engine deliberately halted.**
2. Turbine will not engage below 14 m/s.
3. Bay doors cannot be commanded open with the pod stowed.
4. Pylon over-centre lock holds with the servo unpowered.

### Launch and recovery
Hand launch or bungee, both pods stowed, flaps at −6 mm. There is no landing
gear: land on the belly skid, wings level, both pods stowed, into wind. Approach
with the turbine as the airbrake only above 25 m AGL — below that it must be up.

---

## 6. Regenerating everything

```bash
python3 tools/build_model.py   # models/: STL, OBJ, geometry.json
python3 tools/analysis.py      # docs/ANALYSIS.md
python3 tools/render.py        # docs/img/: three-views
```

All three read `tools/config.py`. Change a dimension there and everything —
model, drawings, mass budget, polar, energy budget, detection performance —
regenerates consistently.
