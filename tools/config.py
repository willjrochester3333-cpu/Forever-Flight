"""
Forever-Flight FF-1 "Ember" -- single source of truth for the airframe.

Every dimension the geometry builder, the performance model, the build manual
and the web viewer use comes from here. Change a number here and everything
downstream regenerates consistently.

Units: millimetres, degrees, grams, watts, seconds unless a name says otherwise.
Axes:  +X aft (nose at x=0), +Y starboard, +Z up.
"""

from __future__ import annotations

# ==========================================================================
# 1. WING
# ==========================================================================

WING = {
    "span": 2000.0,
    # Three-segment planform. Each station: (y, chord, le_x, dihedral_deg,
    # twist_deg). Dihedral and twist are cumulative from the previous station.
    "stations": [
        # y        chord   le_x    dihedral  twist
        (0.0,      195.0,  215.0,  0.0,      0.0),    # centreline
        (450.0,    195.0,  215.0,  0.0,      0.0),    # dihedral / panel break
        (850.0,    160.0,  215.0,  4.5,     -1.4),
        (960.0,    128.0,  215.0,  4.5,     -1.8),
        (1000.0,    92.0,  215.0,  4.5,     -2.0),    # rounded tip
    ],
    "airfoil": {
        "name": "FF-SC1",
        "thickness": 0.098,       # NACA-4 base t/c before flattening
        "camber": 0.029,          # NACA-4 base max camber
        "camber_pos": 0.42,
        "solar_band": (0.19, 0.90),   # chordwise band held flat for cells
        "min_radius_mm": 400.0,       # upper-surface curvature limit
        "te_gap": 0.004,              # blunt TE, fraction of chord
        "dat_file": None,             # e.g. "airfoils/sd7037.dat" to override
    },
    "root_z": 30.0,               # shoulder wing, chord plane above datum
    "incidence": 1.5,             # deg, wing chord vs fuselage datum
    "spar_at_chord": 0.30,        # main spar cap centreline
    "aileron": {"y0": 520.0, "y1": 940.0, "chord_frac": 0.26},
    "flap":    {"y0": 120.0, "y1": 500.0, "chord_frac": 0.26},
}

# ==========================================================================
# 2. SOLAR ARRAY
# ==========================================================================
# Maxeon/SunPower C60-class cells, laser-cut into three strips each. The long
# 125 mm cell edge runs SPANWISE, so the only curvature a strip has to follow
# is the 41.7 mm chordwise one -- hence the flattened aerofoil band.

SOLAR = {
    "cell_full_mm": 125.0,
    "strips_per_cell": 3,
    "strip_w_mm": 125.0 / 3.0,        # 41.67 chordwise
    "strip_l_mm": 125.0,              # spanwise
    "cell_thickness_um": 165.0,
    "cell_eff_stc": 0.225,
    "strip_vmp": 0.574,
    "strip_imp": 5.83 / 3.0,
    "chord_start": 0.19,              # first strip leading edge, fraction of c
    "n_strips_chordwise": 3,
    "span_stations": 13,              # 125 mm bays each side of centre + centre
    "centre_gap_mm": 118.0,           # fuselage/joiner, no cells
    "tip_margin_mm": 62.0,
    "strings": 3,                     # independent MPPT channels
    "encapsulation_loss": 0.94,       # ETFE + adhesive transmission
    "temp_loss": 0.94,                # ~25 C above ambient in flight
    "mismatch_soiling": 0.95,
    "mppt_eff": 0.955,
    "attitude_factor": 0.86,          # bank/heading cosine losses, orbiting
}

# ==========================================================================
# 3. FUSELAGE (pod and boom)
# ==========================================================================

FUSELAGE = {
    # (x, width, height, z_centre) -- superellipse cross-sections
    "stations": [
        (0.0,     18.0,  18.0,   6.0),
        (30.0,    52.0,  56.0,   0.0),
        (65.0,    68.0,  76.0,  -1.0),
        (120.0,   78.0,  92.0,  -2.0),
        (200.0,   80.0,  96.0,  -2.0),
        (300.0,   76.0,  90.0,  -1.0),
        (380.0,   66.0,  80.0,   1.0),
        (470.0,   54.0,  66.0,   3.0),
        (560.0,   40.0,  46.0,   5.0),
        (640.0,   30.0,  32.0,   6.0),
        (820.0,   24.0,  24.0,   7.0),
        (1000.0,  20.0,  20.0,   8.0),
        (1120.0,  17.0,  17.0,   8.0),
    ],
    "superellipse_n": 2.6,
    "turret": {"x": 40.0, "r": 31.0, "z": -34.0},   # sensor ball, below nose
    "bays": {
        "sensor":    (0.0, 65.0),
        "payload":   (65.0, 160.0),
        "battery":   (160.0, 235.0),
        "sparbox":   (250.0, 300.0),
        "avionics":  (300.0, 380.0),
        "motor_bay": (385.0, 790.0),     # dorsal spine, pylon + folded blades
        "rat_bay":   (350.0, 575.0),     # ventral, arm + folded turbine blades
    },
    # Dorsal spine that swallows the stowed pylon. (x, half_width, height
    # above the local fuselage deck).
    "dorsal_spine": [
        (370.0,  6.0,   4.0),
        (400.0, 23.0,  40.0),
        (520.0, 24.0,  44.0),
        (640.0, 20.0,  40.0),
        (740.0, 13.0,  24.0),
        (820.0,  5.0,   4.0),
    ],
    # Ventral fairing over the stowed turbine arm. (x, half_width, depth below
    # the local fuselage keel).
    "ventral_fairing": [
        (345.0,  5.0,   3.0),
        (375.0, 18.0,  26.0),
        (470.0, 19.0,  30.0),
        (545.0, 15.0,  26.0),
        (585.0,  5.0,   4.0),
    ],
}

# ==========================================================================
# 4. V-TAIL
# ==========================================================================

VTAIL = {
    "le_x": 1050.0,
    "root_chord": 140.0,
    "tip_chord": 95.0,
    "panel_span": 300.0,          # along the panel, not projected
    "dihedral": 38.0,             # deg from horizontal (76 deg included)
    "incidence": -1.0,
    "thickness": 0.09,            # symmetric section
    "sweep_le": 8.0,
    "root_z": 10.0,
    "ruddervator_chord_frac": 0.33,
}

# ==========================================================================
# 5. RETRACTABLE MOTOR PYLON (dorsal)
# ==========================================================================

MOTOR = {
    # 4-bar pivot on the fuselage deck at the FRONT of the dorsal bay, so the
    # pylon folds aft into the spine and the deployed prop disc sits well
    # behind the wing trailing edge.
    "pivot": (400.0, 0.0, 10.0),
    "pylon_len": 190.0,
    "pylon_chord": 74.0,
    "pylon_thickness": 0.12,          # symmetric strut section
    "deployed_angle": 80.0,           # deg from +X toward +Z (aft-leaning)
    "stowed_angle": 5.0,
    "nacelle_len": 74.0,
    "nacelle_dia": 38.0,
    "nacelle_offset": 56.0,           # hub aft of the pylon top (pusher)
    "prop_dia": 254.0,                # 10 x 6 folding, pusher
    "prop_pitch_in": 6.0,
    "blades": 2,
    "spinner_len": 30.0,
    "spinner_dia": 34.0,
    "thrust_angle": -1.5,             # deg, down-thrust
    "motor_kv": 500,                  # on 3S; 480-540 acceptable
}

# ==========================================================================
# 6. RETRACTABLE ENERGY-RECOVERY TURBINE / "dynamo" (ventral)
# ==========================================================================

RAT = {
    # A 110 mm rotor only makes ~1.3 W at cruise speed -- power goes as V^3 and
    # as R^2, so the turbine was resized. 150 mm folding rotor, stowed by
    # swinging FORWARD under the belly with the blades folded along the arm.
    "pivot": (560.0, 0.0, -20.0),
    "arm_len": 130.0,
    "arm_chord": 46.0,
    "arm_thickness": 0.14,
    "deployed_angle": -78.0,          # deg from +X toward +Z (down and aft)
    "stowed_angle": 176.0,            # arm lies forward along the keel
    "nacelle_len": 62.0,
    "nacelle_dia": 30.0,
    "rotor_dia": 150.0,
    "hub_r": 15.0,
    "blades": 4,                      # folding, centrifugally opened
    "blade_chord": 18.0,
    "solidity": 0.306,                # B*c/(pi*R)
    "lambda_design": 2.5,             # tip-speed ratio
    "cp": 0.36,                       # rotor power coefficient at lambda 2.5
    "spinner_dia": 26.0,
    "spinner_len": 20.0,
    "gen_kv": 380,                    # 350-420 acceptable
}

# ==========================================================================
# 6b. VERTICAL DEPLOYMENT MAST (VDM-1) -- alternative to the swing arm
# ==========================================================================
# Telescoping vertical mast driven by one micro linear actuator. Geometry is
# solved by tools/mast.py, which is the authority; the values it needs are
# here and the derived ones (stroke, stage travel, sections) come from it at
# build time so the two cannot drift apart.

RAT_MOUNT = "mast"          # "swing" (baseline pivoting arm) or "mast"

RAT_MAST = {
    "station_x": 425.0,
    "stages": 3,
    "trunk_x": 34.0,            # fore-aft: the air load is fore-aft
    "trunk_y": 22.0,
    "wall": 0.8,
    "nest_gap": 1.0,
    "actuator": {
        "model": "L12-50-210:1 class",
        "stroke_mm": 50.0, "used_mm": 42.7, "force_N": 80.0,
        "speed_mm_s": 6.0, "mass_g": 40.0, "body_mm": 101.0,
        "x0": 470.0, "z": 2.0,          # body runs aft from here
        "bellcrank_ratio": 1.0,
        "pivot": (447.0, 0.0, 6.0), "arm_mm": 30.0,
    },
    "reduction": "rack-pinion-rack doublers, 1:2:3",
    "bushing_mu": 0.12,
    "v_retract_max": 20.0,      # m/s -- above this, do not move the mast
    "detent": "2 x 4 N magnetic, stowed",
}

# Ventral fairing for the mast variant: shorter and slightly deeper than the
# swing-arm blister, because it only has to cover the stowed rotor.
RAT_MAST_FAIRING = [
    (352.0,  5.0,   3.0),
    (382.0, 22.0,  30.0),
    (425.0, 25.0,  36.0),
    (470.0, 21.0,  31.0),
    (515.0,  6.0,   4.0),
]

# ==========================================================================
# 7. MASS BUDGET (grams)
# ==========================================================================
# Every line is an installed mass: part + adhesive + fasteners + its share of
# wiring. "unc" is the 1-sigma uncertainty used for the growth allowance.

MASS = [
    # group,        item,                                    g,     unc
    ("Structure",   "Wing: spar caps, webs, D-box, ribs",   168.0,  18.0),
    ("Structure",   "Wing: skins, ETFE, joiner tube",       142.0,  16.0),
    ("Structure",   "Wing: control surfaces + hinges",       34.0,   5.0),
    ("Structure",   "Fuselage pod (Kevlar/carbon)",          88.0,  10.0),
    ("Structure",   "Tail boom (carbon, tapered)",           38.0,   4.0),
    ("Structure",   "V-tail assembly",                       46.0,   6.0),
    ("Propulsion",  "Motor (outrunner, 480 Kv class)",       52.0,   4.0),
    ("Propulsion",  "ESC (bidirectional, 30 A)",             19.0,   2.0),
    ("Propulsion",  "Folding prop + spinner + hub",          27.0,   3.0),
    ("Propulsion",  "Pylon, 4-bar, over-centre lock",        58.0,   8.0),
    ("Propulsion",  "Pylon actuator + bay doors",            31.0,   5.0),
    ("Recovery",    "Turbine rotor + hub",                   18.0,   3.0),
    ("Recovery",    "Generator (low-Kv, custom wound)",      44.0,   5.0),
    ("Recovery",    "Regen controller (FOC)",                26.0,   3.0),
    # VDM-1 vertical mast (RAT_MOUNT = "mast"). Swap this block for the
    # single 38 g swing-arm line to price the baseline.
    ("Recovery",    "Mast: fixed trunk + bulkhead doublers",  16.0,   3.0),
    ("Recovery",    "Mast: 3 telescoping stages",             32.0,   5.0),
    ("Recovery",    "Mast: pinions, bushings, racks",         10.0,   2.0),
    ("Recovery",    "Micro linear actuator + mount",          46.0,   4.0),
    ("Recovery",    "Bellcrank, con-rod, pivots",              9.0,   2.0),
    ("Recovery",    "Bay doors, cam, wiper seal",             14.0,   3.0),
    ("Recovery",    "End sensors, magnets, wiring",            9.0,   2.0),
    ("Energy",      "Li-ion 3S1P 21700 (5.0 Ah cells)",     212.0,   6.0),
    ("Energy",      "BMS, fuse, pack wiring, tray",          27.0,   4.0),
    ("Energy",      "Solar cells (installed)",              104.0,   8.0),
    ("Energy",      "Encapsulation, buses, bypass diodes",   42.0,   6.0),
    ("Energy",      "3-channel MPPT",                        34.0,   4.0),
    ("Avionics",    "Flight controller + IMU + baro",        21.0,   2.0),
    ("Avionics",    "GNSS + compass",                        11.0,   1.0),
    ("Avionics",    "Control link RX + antenna",             12.0,   2.0),
    ("Avionics",    "Servos (4 x 8 g wing/tail)",            32.0,   3.0),
    ("Avionics",    "Airframe wiring, connectors",           28.0,   6.0),
    ("Payload",     "LWIR radiometric core + lens",          14.0,   2.0),
    ("Payload",     "Visible camera module",                  9.0,   1.0),
    ("Payload",     "Edge compute board + shield",           16.0,   2.0),
    ("Payload",     "LoRa/telemetry radio + antenna",        17.0,   2.0),
    ("Payload",     "Sensor turret shell + damping",         24.0,   4.0),
]

MASS_GROWTH_ALLOWANCE = 0.06     # applied on top of the itemised total

# ==========================================================================
# 8. AERODYNAMIC DRAG BUILD-UP  (all referenced to wing area S)
# ==========================================================================

DRAG = {
    "wing_cd_min": 0.0110,        # section profile drag at design CL, Re~120k
    "wing_cd_k": 0.0100,          # Cd_p = cd_min + k*(CL - CL_design)^2
    "wing_cl_design": 0.72,
    "components": [               # (name, delta CD0, note)
        ("Fuselage pod + boom",     0.00205, "S_wet 0.078 m2, Cf 0.0060, FF 1.35"),
        ("V-tail",                  0.00200, "S 0.0705 m2, Cd_p 0.0102"),
        ("Sensor turret",           0.00095, "faired ball, 62 mm"),
        ("Interference + gaps",     0.00190, "hinge lines, wing root, antennas"),
        ("Solar cell steps",        0.00120, "cell edges, bus ribbons under ETFE"),
        ("Dorsal motor-bay fairing", 0.00110, "spine, x 370-800"),
        ("Ventral turbine fairing",  0.00075, "keel blister, x 345-585"),
        ("Bay door seams",           0.00055, "four doors, taped edges"),
    ],
    "margin": 0.15,               # uncertainty margin on the sum
    "oswald_e": 0.88,
    "cl_max": 1.25,               # with the solar surface, no turbulator tape
    "cl_max_flapped": 1.55,
    # Deployment penalties (delta CD0, referenced to S)
    "motor_deployed_off": 0.00250,   # pylon + nacelle + folded blades
    "rat_deployed_feathered": 0.00330,   # 150 mm rotor, arm + nacelle
}

# ==========================================================================
# 9. PROPULSION / ELECTRICAL EFFICIENCIES
# ==========================================================================

EFF = {
    "prop": 0.68,                 # folding prop at cruise J, low Re
    "motor": 0.84,
    "esc": 0.95,
    "turbine_cp": 0.35,           # rotor power coefficient (Betz limit 0.593)
    "generator": 0.72,
    "rectifier_charger": 0.90,
    "battery_roundtrip": 0.94,
}

# ==========================================================================
# 10. ENERGY / HOTEL LOADS (watts, flight average)
# ==========================================================================

HOTEL = [
    ("Flight controller + IMU + baro",  1.10),
    ("GNSS + compass",                  0.35),
    ("Control link RX",                 0.30),
    ("Servos (soaring duty)",           1.60),
    ("LWIR core",                       0.65),
    ("Visible camera",                  0.60),
    ("Edge compute (50% duty)",         2.40),
    ("Telemetry radio (avg)",           0.35),
    ("MPPT + BMS quiescent",            0.45),
    ("Deployment actuators (avg)",      0.10),
]

BATTERY = {
    "cells_series": 3,
    "cells_parallel": 1,
    "cell_wh": 17.6,              # 21700, 4.9 Ah at 3.6 V nominal
    "usable_dod": 0.85,
    "reserve_wh": 6.0,            # held back for approach + go-around
}

# ==========================================================================
# 11. MISSION / ENVIRONMENT
# ==========================================================================

MISSION = {
    "target_endurance_h": 8.0,
    "cruise_alt_agl_m": 400.0,
    "soar_band_m": (250.0, 900.0),
    "rho_cruise": 1.170,          # kg/m3 at ~400 m ISA, warm day
    "rho_sl": 1.225,
    "nu": 1.50e-5,                # kinematic viscosity, m2/s
    "g": 9.80665,
    "sites": [
        # name, latitude, day-of-year, sky clearness, peak thermal strength
        # (m/s), peak fraction of time the aircraft can hold usable lift
        ("S. Europe / California, mid-Aug",  38.0, 227, 0.78, 3.2, 0.42),
        ("UK / N. Europe, late June",        51.5, 172, 0.68, 2.2, 0.34),
    ],
    # Thermal day: both strength and availability follow a bell from t0 to t1.
    "thermal_day": {"t0": 9.5, "t1": 18.5, "shape": 0.7},
}

PAYLOAD_SENSOR = {
    "name": "LWIR radiometric microbolometer, 160 x 120",
    "hfov_deg": 57.0,
    "h_pixels": 160,
    "v_pixels": 120,
    "band_um": (8.0, 14.0),
    "netd_mk": 50.0,
    "high_gain_max_c": 140.0,
    "low_gain_max_c": 450.0,
    "background_c": 27.0,
    "flame_temp_k": 800.0,
    "test_fire_areas_m2": [0.05, 0.1, 0.5, 1.0, 4.0],
}
