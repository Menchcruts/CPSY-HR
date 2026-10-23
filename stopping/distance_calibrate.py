#!/usr/bin/env python3
"""Calibrate the distance sensor: measure its voltage at known distances.

    python3 distance_calibrate.py            # adds to distance_table.csv
    python3 distance_calibrate.py --fresh    # starts a new table

Put a flat, matte target (cardboard, a wall) square in front of the sensor,
measure the gap from the FRONT OF THE ROVER (not the sensor itself), type it
in and press Enter. Measuring from the front means the table directly gives
the gap the rover has left, however far back the sensor is mounted.

Typing a distance you already measured replaces that point. An empty line
finishes. The table is saved after every point, to distance_table.csv next to
these scripts, and a timestamped copy goes to logs/ for plot_logs.py.
"""
import argparse
import csv
import shutil
import statistics
import time
from pathlib import Path

from distance_sensor import TABLE_PATH, VREF, usable_part

import numpy as np

OLD_A, OLD_N = 56.351, -1.186       # your previous formula: A * v**n
# The GP2Y0A02YK is rated for 20-150 cm; 10 and 15 show where its peak is.
SUGGESTED = "10 15 20 25 30 40 50 60 70 80 100 120 150"


def old_formula(v):
    return OLD_A * v ** OLD_N if v > 0 else float("inf")


def measure(adc, seconds):
    """Median and spread of the voltage over `seconds` of reads."""
    values = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        values.append(adc.value * VREF)
        time.sleep(0.005)
    return statistics.median(values), statistics.pstdev(values), len(values)


def load(path):
    if not path.exists():
        return {}
    with path.open(newline="") as f:
        return {float(r["distance_cm"]): r for r in csv.DictReader(f)}


def save(path, points):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["distance_cm", "voltage", "voltage_std", "samples"])
        writer.writeheader()
        for d in sorted(points):
            writer.writerow(points[d])


def summary(points):
    d = np.array(sorted(points))
    v = np.array([float(points[x]["voltage"]) for x in d])
    print(f"\n{'distance':>9} {'voltage':>9} {'± noise':>8} {'old formula':>12}")
    for dist, volt in zip(d, v):
        std = float(points[dist]["voltage_std"])
        old = old_formula(volt)
        print(f"{dist:7g}cm {volt:8.3f}V {std:7.3f}V {old:8.1f}cm ({old - dist:+.1f})")
    if len(d) < 2:
        return
    kept, _, dropped = usable_part(d, v)
    if len(kept) >= 2:
        print(f"\nusable range: {kept.min():g}-{kept.max():g} cm")
    if dropped:
        listed = ", ".join(f"{x:g}" for x in dropped)
        print(f"left out {listed} cm: there the voltage doesn't fall steadily with distance.")
        closer = [x for x in dropped if x < kept.min()]
        if closer:
            print("  Points closer than the voltage peak are inside the sensor's minimum range:")
            print("  an object that close reads as FARTHER away, so the rover must stop before it.")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--channel", type=int, default=0, help="MCP3008 channel (default 0)")
    p.add_argument("--seconds", type=float, default=1.0,
                   help="how long to read at each distance (default 1)")
    p.add_argument("--fresh", action="store_true", help="start a new table")
    args = p.parse_args()

    from gpiozero import MCP3008
    adc = MCP3008(channel=args.channel)
    points = {} if args.fresh else load(TABLE_PATH)
    if points:
        print(f"adding to the existing table ({len(points)} points); --fresh starts over")
    print(f"suggested distances (cm): {SUGGESTED}")
    print("stop going farther once the voltage barely changes: that's the sensor's range.\n")

    try:
        while True:
            answer = input("distance from the rover's front in cm (Enter = finish): ").strip()
            if not answer:
                break
            try:
                dist = float(answer)
            except ValueError:
                print("  type a number, e.g. 30")
                continue
            volt, std, n = measure(adc, args.seconds)
            points[dist] = {"distance_cm": f"{dist:g}", "voltage": f"{volt:.4f}",
                            "voltage_std": f"{std:.4f}", "samples": n}
            save(TABLE_PATH, points)
            old = old_formula(volt)
            print(f"  {dist:g} cm: {volt:.3f} V (± {std:.3f} V over {n} reads), "
                  f"old formula says {old:.1f} cm")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        adc.close()

    if not points:
        print("nothing measured")
        return
    summary(points)
    Path("logs").mkdir(exist_ok=True)
    copy = Path("logs") / f"{time.strftime('%Y%m%d-%H%M%S')}_distance_cal.csv"
    shutil.copyfile(TABLE_PATH, copy)
    print(f"\nsaved {TABLE_PATH} (copy for plotting: {copy})")


if __name__ == "__main__":
    main()
