#!/usr/bin/env python3
"""
Flight log readers for the FF-1.

Three input formats, no dependencies:

  .BIN   ArduPilot dataflash straight off the SD card (self-describing)
  .log   ArduPilot text log (Mission Planner "convert to text")
  .csv   anything with a time column -- headers are matched fuzzily

Everything lands in one Log object holding named channels on a common clock,
then `resample()` puts them on a uniform time base so the analysis can assume
evenly spaced samples.

Channel names are normalised to the canonical set in CHANNELS. Whatever your
log calls airspeed, it comes out as "airspeed" in m/s.
"""

from __future__ import annotations

import csv
import math
import os
import struct
import sys

# --------------------------------------------------------------------------
# canonical channels
# --------------------------------------------------------------------------
# name          unit     what the analysis does with it
CHANNELS = {
    "alt":        ("m",   "altitude, baro or GPS; climb rate is differentiated from it"),
    "airspeed":   ("m/s", "indicated airspeed -- REQUIRED for the drag polar fit"),
    "groundspeed": ("m/s", "fallback if no pitot; wind makes the polar fit unreliable"),
    "climb":      ("m/s", "logged vario if present, else derived from alt"),
    "roll":       ("deg", "used to detect circling and to reject banked samples"),
    "pitch":      ("deg", ""),
    "throttle":   ("0-1", "used to separate powered flight from gliding"),
    "volt":       ("V",   "pack voltage"),
    "curr":       ("A",   "pack current, positive = discharge"),
    "used_mah":   ("mAh", "cumulative consumption if the log has it"),
    "solar_w":    ("W",   "MPPT output -- FF-1 specific, log it"),
    "turbine_w":  ("W",   "turbine controller output -- FF-1 specific, log it"),
    "mast_mm":    ("mm",  "turbine mast extension -- FF-1 specific"),
    "temp":       ("degC", "outside air temperature, for density correction"),
    "lat":        ("deg", ""),
    "lon":        ("deg", ""),
}

# Aliases, lowercased and stripped of non-alphanumerics before matching.
ALIASES = {
    "alt": ["alt", "altitude", "baroalt", "baroaltitude", "gpsalt", "altmsl",
            "altagl", "relhomealt", "altitudem", "ctunalt", "posalt", "hgt"],
    "airspeed": ["airspeed", "arspairspeed", "ias", "tas", "as", "ctunas",
                 "airspeedms", "aspd"],
    "groundspeed": ["groundspeed", "gspd", "gpsspd", "spd", "speed", "gs",
                    "velocity", "gpsgroundspeed"],
    "climb": ["climb", "climbrate", "vario", "vz", "vspeed", "verticalspeed",
              "ctunclimbrate", "netto", "soarnetto"],
    "roll": ["roll", "attroll", "rolldeg", "phi"],
    "pitch": ["pitch", "attpitch", "pitchdeg", "theta"],
    "throttle": ["throttle", "thr", "ctunthro", "thro", "throut", "rcouc3"],
    "volt": ["volt", "voltage", "batvolt", "vbat", "battvoltage", "battvolt"],
    "curr": ["curr", "current", "batcurr", "amps", "battcurrent", "current1"],
    "used_mah": ["currtot", "usedmah", "mah", "consumed", "battconsumed",
                 "batcurrtot"],
    "solar_w": ["solarw", "solar", "mpptw", "pvw", "solarpower", "mppt"],
    "turbine_w": ["turbinew", "turbine", "ratw", "ratpower", "dynamow", "regenw"],
    "mast_mm": ["mastmm", "mast", "mastext", "mastextension"],
    "temp": ["temp", "temperature", "baroTemp", "oat", "airtemp", "tempc"],
    "lat": ["lat", "latitude", "gpslat"],
    "lon": ["lon", "lng", "longitude", "gpslon", "gpslng"],
}

_LOOKUP = {}
for canon, names in ALIASES.items():
    for n in names:
        _LOOKUP[n.lower()] = canon


def _key(s):
    return "".join(ch for ch in str(s).lower() if ch.isalnum())


def match_channel(header):
    """Map a log column name onto a canonical channel, or None."""
    k = _key(header)
    if k in _LOOKUP:
        return _LOOKUP[k]
    # try the last dotted component, e.g. "BAT.Volt" -> "volt"
    if "." in str(header):
        k2 = _key(str(header).rsplit(".", 1)[-1])
        if k2 in _LOOKUP:
            return _LOOKUP[k2]
    return None


# --------------------------------------------------------------------------
# Log container
# --------------------------------------------------------------------------

class Log:
    def __init__(self, source=""):
        self.source = source
        self.raw: dict = {}        # channel -> list of (t_seconds, value)
        self.notes: list = []

    def add(self, channel, t, value):
        if value is None:
            return
        try:
            v = float(value)
        except (TypeError, ValueError):
            return
        if v != v or abs(v) == float("inf"):      # NaN / inf
            return
        self.raw.setdefault(channel, []).append((float(t), v))

    def finish(self):
        for ch in self.raw:
            self.raw[ch].sort(key=lambda p: p[0])
        return self

    def span(self):
        ts = [p[0] for s in self.raw.values() for p in s]
        return (min(ts), max(ts)) if ts else (0.0, 0.0)

    def present(self):
        return sorted(k for k, v in self.raw.items() if len(v) > 1)

    def summary(self):
        t0, t1 = self.span()
        out = [f"source: {os.path.basename(self.source)}",
               f"duration: {(t1 - t0) / 60:.1f} min ({t1 - t0:.0f} s)"]
        for ch in self.present():
            s = self.raw[ch]
            vals = [v for _t, v in s]
            out.append(f"  {ch:<12s} {len(s):>7d} samples  "
                       f"{min(vals):>9.2f} .. {max(vals):>9.2f} {CHANNELS.get(ch, ('', ''))[0]}")
        missing = [c for c in ("alt", "airspeed") if c not in self.raw]
        if missing:
            out.append(f"  MISSING (analysis will be limited): {', '.join(missing)}")
        return "\n".join(out)

    # -- resampling -------------------------------------------------
    def resample(self, dt=0.25, channels=None, max_gap=5.0):
        """Uniform time base. Linear interpolation, holes left as None.

        A hole wider than max_gap is not bridged -- interpolating across a
        30 s logging dropout would invent glides that never happened.
        """
        t0, t1 = self.span()
        if t1 <= t0:
            return {"t": [], "dt": dt}
        n = int((t1 - t0) / dt) + 1
        grid = [t0 + i * dt for i in range(n)]
        out = {"t": grid, "dt": dt, "t0": t0}
        for ch in (channels or self.present()):
            series = self.raw.get(ch)
            if not series or len(series) < 2:
                continue
            col, j = [], 0
            for tg in grid:
                while j + 1 < len(series) and series[j + 1][0] < tg:
                    j += 1
                ta, va = series[j]
                if j + 1 >= len(series):
                    col.append(va if abs(tg - ta) <= max_gap else None)
                    continue
                tb, vb = series[j + 1]
                if tb - ta > max_gap:
                    col.append(va if abs(tg - ta) <= dt else None)
                elif tb == ta:
                    col.append(va)
                else:
                    f = (tg - ta) / (tb - ta)
                    f = max(0.0, min(1.0, f))
                    col.append(va + f * (vb - va))
            out[ch] = col
        return out


# --------------------------------------------------------------------------
# CSV
# --------------------------------------------------------------------------

TIME_KEYS = ["timeus", "timems", "time", "timestamp", "t", "seconds", "sec",
             "elapsed", "timesec", "millis", "micros", "clock"]


def read_csv(path):
    log = Log(path)
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as fh:
        sample = fh.read(8192)
        fh.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel
        rdr = csv.reader(fh, dialect)
        header = None
        for row in rdr:
            if row and any(_key(c) for c in row):
                header = row
                break
        if not header:
            raise ValueError(f"{path}: no header row found")

        tcol, tscale = None, 1.0
        for i, h in enumerate(header):
            k = _key(h)
            if k in TIME_KEYS:
                tcol = i
                tscale = (1e-6 if k in ("timeus", "micros") else
                          1e-3 if k in ("timems", "millis") else 1.0)
                break
        if tcol is None:
            log.notes.append("no time column found; using row index as seconds")

        cols = {}
        for i, h in enumerate(header):
            if i == tcol:
                continue
            c = match_channel(h)
            if c:
                cols.setdefault(c, i)

        if not cols:
            raise ValueError(f"{path}: no recognised channels in header: {header}")
        log.notes.append("columns used: " + ", ".join(
            f"{header[i]} -> {c}" for c, i in sorted(cols.items())))

        for r, row in enumerate(rdr):
            if not row:
                continue
            try:
                t = float(row[tcol]) * tscale if tcol is not None else float(r)
            except (ValueError, IndexError):
                continue
            for c, i in cols.items():
                if i < len(row):
                    log.add(c, t, row[i])
    return log.finish()


# --------------------------------------------------------------------------
# ArduPilot text log  (Mission Planner "convert .bin to .log")
# --------------------------------------------------------------------------
# Lines look like:  ATT, 12345678, 0.1, 2.3, ...
# preceded by:      FMT, 30, 43, ATT, QccccCC, TimeUS,DesRoll,Roll,...

def read_ardupilot_log(path):
    log = Log(path)
    fmts = {}
    used = set()
    with open(path, errors="replace") as fh:
        for line in fh:
            parts = [p.strip() for p in line.split(",")]
            if not parts:
                continue
            if parts[0] == "FMT" and len(parts) >= 6:
                fmts[parts[3]] = parts[5:]
                continue
            labels = fmts.get(parts[0])
            if not labels:
                continue
            vals = parts[1:]
            tmap = {l.lower(): i for i, l in enumerate(labels)}
            ti = tmap.get("timeus", tmap.get("timems"))
            if ti is None or ti >= len(vals):
                continue
            try:
                t = float(vals[ti]) * (1e-6 if "timeus" in tmap else 1e-3)
            except ValueError:
                continue
            for i, lab in enumerate(labels):
                if i >= len(vals) or i == ti:
                    continue
                c = match_channel(f"{parts[0]}.{lab}") or match_channel(lab)
                if c:
                    log.add(c, t, vals[i])
                    used.add(f"{parts[0]}.{lab}->{c}")
    log.notes.append("fields used: " + ", ".join(sorted(used)[:24]))
    return log.finish()


# --------------------------------------------------------------------------
# ArduPilot .BIN dataflash
# --------------------------------------------------------------------------

HEAD = b"\xa3\x95"
FMT_TYPE = 0x80

# format char -> (struct code, size, multiplier)
_FC = {
    "b": ("b", 1, 1.0), "B": ("B", 1, 1.0),
    "h": ("h", 2, 1.0), "H": ("H", 2, 1.0),
    "i": ("i", 4, 1.0), "I": ("I", 4, 1.0),
    "f": ("f", 4, 1.0), "d": ("d", 8, 1.0),
    "q": ("q", 8, 1.0), "Q": ("Q", 8, 1.0),
    "c": ("h", 2, 0.01), "C": ("H", 2, 0.01),
    "e": ("i", 4, 0.01), "E": ("I", 4, 0.01),
    "L": ("i", 4, 1e-7), "M": ("B", 1, 1.0),
    "n": ("4s", 4, None), "N": ("16s", 16, None),
    "Z": ("64s", 64, None), "a": ("32h", 64, None),
}


def _fmt_struct(fmt):
    codes, mults, size = "<", [], 0
    for ch in fmt:
        if ch not in _FC:
            return None, None, None
        code, sz, mul = _FC[ch]
        codes += code
        mults.append(mul)
        size += sz
    return codes, mults, size


def read_bin(path):
    log = Log(path)
    data = open(path, "rb").read()
    defs = {}            # msg type -> (name, labels, struct, mults, size)
    used, skipped = set(), {}
    i, n = 0, len(data)

    while i + 3 <= n:
        if data[i:i + 2] != HEAD:
            j = data.find(HEAD, i + 1)       # resync
            if j < 0:
                break
            i = j
            continue
        mtype = data[i + 2]
        body = i + 3

        if mtype == FMT_TYPE:
            if body + 86 > n:
                break
            t, ln = data[body], data[body + 1]
            name = data[body + 2:body + 6].split(b"\0")[0].decode("ascii", "replace")
            fmt = data[body + 6:body + 22].split(b"\0")[0].decode("ascii", "replace")
            labs = data[body + 22:body + 86].split(b"\0")[0].decode("ascii", "replace")
            codes, mults, size = _fmt_struct(fmt)
            if codes and size == ln - 3:
                defs[t] = (name, labs.split(","), codes, mults, size)
            i = body + 86
            continue

        d = defs.get(mtype)
        if d is None:
            skipped[mtype] = skipped.get(mtype, 0) + 1
            j = data.find(HEAD, i + 1)
            if j < 0:
                break
            i = j
            continue

        name, labels, codes, mults, size = d
        if body + size > n:
            break
        try:
            vals = struct.unpack(codes, data[body:body + size])
        except struct.error:
            i = body
            continue
        i = body + size

        row = {}
        for lab, v, mul in zip(labels, vals, mults):
            if mul is None:
                continue
            row[lab] = v * mul
        t = row.get("TimeUS")
        if t is None:
            continue
        t *= 1e-6
        for lab, v in row.items():
            if lab == "TimeUS":
                continue
            c = match_channel(f"{name}.{lab}") or match_channel(lab)
            if c:
                log.add(c, t, v)
                used.add(f"{name}.{lab}->{c}")

    log.notes.append(f"{len(defs)} message types defined; "
                     f"{len(skipped)} unknown types skipped")
    log.notes.append("fields used: " + ", ".join(sorted(used)[:24]))
    return log.finish()


# --------------------------------------------------------------------------

def read(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".bin":
        return read_bin(path)
    if ext in (".log", ".txt"):
        return read_ardupilot_log(path)
    if ext in (".csv", ".tsv", ""):
        return read_csv(path)
    raise ValueError(f"unrecognised log type: {ext} (expected .bin, .log or .csv)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        print("\nCanonical channels:")
        for k, (u, why) in CHANNELS.items():
            print(f"  {k:<12s} {u:<6s} {why}")
        sys.exit(0)
    lg = read(sys.argv[1])
    print(lg.summary())
    for nt in lg.notes:
        print("note:", nt)
