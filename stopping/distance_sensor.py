#!/usr/bin/env python3
"""Distance from the GP2Y0A02YK IR sensor (on an MCP3008), via a calibration table.

Instead of one formula for the whole range, this converts voltage to distance
by interpolating between the voltages you measured at known distances with
distance_calibrate.py (saved in distance_table.csv). It also filters the noise:
the median of a few quick reads, then a short running median over recent calls.

    sensor = DistanceSensor(channel=0)
    cm = sensor.distance()     # float("inf") when nothing is within range

Run it directly for a live readout, like your distance.py:

    python3 distance_sensor.py
"""
import csv
import statistics
import time
from collections import deque
from pathlib import Path

import numpy as np

VREF = 3.3   # MCP3008 reference voltage
TABLE_PATH = Path(__file__).with_name("distance_table.csv")   # next to these scripts


def load_table(path):
    """(distances_cm, volts) from distance_table.csv, sorted by distance."""
    with Path(path).open(newline="") as f:
        rows = [(float(r["distance_cm"]), float(r["voltage"])) for r in csv.DictReader(f)]
    rows.sort()
    return np.array([d for d, _ in rows]), np.array([v for _, v in rows])


def usable_part(distances, volts):
    """The part of the table where voltage falls steadily with distance.

    Many IR sensors (Sharp ones especially) have a minimum range: closer than
    that, the voltage drops again, so a very close object looks far away. Only
    the part from the voltage peak outwards can be converted unambiguously.
    Returns (distances, volts, dropped) with dropped = the points left out.
    """
    peak = int(np.argmax(volts))
    keep_d, keep_v = list(distances[peak:]), list(volts[peak:])
    dropped = list(distances[:peak])
    # Drop any later point that doesn't continue the downward trend (noise).
    i = 1
    while i < len(keep_v):
        if keep_v[i] >= keep_v[i - 1]:
            dropped.append(keep_d.pop(i))
            keep_v.pop(i)
        else:
            i += 1
    return np.array(keep_d), np.array(keep_v), sorted(float(x) for x in dropped)


class VoltageToDistance:
    """Interpolates distance from voltage using a calibration table."""

    def __init__(self, distances, volts):
        d, v, self.dropped = usable_part(np.asarray(distances), np.asarray(volts))
        if len(d) < 2:
            raise ValueError("calibration table needs at least 2 usable points")
        order = np.argsort(v)                 # np.interp needs rising x values
        self._v, self._d = v[order], d[order]
        self.min_cm, self.max_cm = float(d.min()), float(d.max())

    def __call__(self, v):
        if v < self._v[0]:
            return float("inf")               # weaker than the farthest point: out of range
        if v > self._v[-1]:
            return self.min_cm                # stronger than the closest usable point
        return float(np.interp(v, self._v, self._d))


class DistanceSensor:
    def __init__(self, channel=0, table=TABLE_PATH, reads=4, window=3):
        """reads: quick ADC reads per call (median), against ADC noise.
        window: running median over the last N calls, against spikes. Each
        extra call in the window adds about one loop period of delay."""
        if not Path(table).exists():
            raise SystemExit(f"no calibration table at {table}: run distance_calibrate.py first")
        from gpiozero import MCP3008
        self.convert = VoltageToDistance(*load_table(table))
        self.adc = MCP3008(channel=channel)
        self.reads = reads
        self._recent = deque(maxlen=window)

    def voltage(self):
        v = statistics.median([self.adc.value for _ in range(self.reads)]) * VREF
        self._recent.append(v)
        return statistics.median(self._recent)

    def distance(self):
        return self.convert(self.voltage())

    def close(self):
        self.adc.close()


if __name__ == "__main__":
    sensor = DistanceSensor()
    lo, hi = sensor.convert.min_cm, sensor.convert.max_cm
    print(f"calibrated range {lo:g}-{hi:g} cm")
    if sensor.convert.dropped:
        print(f"(left out {sensor.convert.dropped} cm: voltage didn't fall steadily there)")
    try:
        while True:
            v = sensor.voltage()
            cm = sensor.convert(v)
            shown = f"beyond {hi:g} cm" if cm == float("inf") else f"{cm:5.1f} cm"
            print(f"voltage {v:.3f} V  ->  {shown}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping")
