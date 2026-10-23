#!/usr/bin/env python3
"""Plot the robot's CSV logs on your desktop instead of the Pi.

    python3 plot_logs.py LOGS_DIR NEW_GRAPHS_DIR OLD_GRAPHS_DIR
    python3 plot_logs.py LOGS_DIR NEW_GRAPHS_DIR OLD_GRAPHS_DIR --force   # replot everything

Plots every CSV in LOGS_DIR that hasn't been plotted before. If there is
anything new, the graphs already in NEW_GRAPHS_DIR are first moved to
OLD_GRAPHS_DIR, so NEW_GRAPHS_DIR only ever holds the latest batch.

What has been plotted is remembered by file content (in OLD_GRAPHS_DIR/.plotted.json),
so renaming a CSV or a graph doesn't make it plot again, but a CSV whose
content changed (say, a sweep you copied over halfway) does.

Understands the PID/open-loop logs from pid_motor_test.py (old and new
column layouts), the sweeps from duty_sweep.py, the obstacle runs from
obstacle_stop.py and the calibration tables from distance_calibrate.py.
Needs matplotlib.
"""

import argparse
import csv
import hashlib
import json
import math
import shutil
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = []
        for raw in reader:
            row = {}
            for key, value in raw.items():
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = None  # empty cell, e.g. no battery reading
            rows.append(row)
        return reader.fieldnames or [], rows


def wheels_in(header):
    return [c[len("speed_") :] for c in header if c.startswith("speed_")]


def plot_timeseries(header, rows, path, title):
    wheels = wheels_in(header)
    has_battery = any(r.get("battery_v") is not None for r in rows)
    has_distance = "distance_cm" in header
    n = 2 + has_distance + has_battery
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(8, 3 * n))
    t = [r["t"] for r in rows]
    for i, w in enumerate(wheels):
        color = f"C{i}"
        # Newer logs have the (ramped) setpoint the PID followed as well as the
        # final target; older ones only the target, or one shared "target".
        if f"setpoint_{w}" in header:
            setpoint_col = f"setpoint_{w}"
        elif f"target_{w}" in header:
            setpoint_col = f"target_{w}"
        else:
            setpoint_col = "target"
        duty_col = f"duty_{w}"
        setpoints = [r.get(setpoint_col) for r in rows]
        duties = [r.get(duty_col) for r in rows]
        # Open-loop runs log the duty as the "target"; only draw real setpoints.
        open_loop = all(
            a is not None and b is not None and math.isclose(a, b)
            for a, b in zip(setpoints, duties)
        )
        if setpoint_col in header and not open_loop:
            axes[0].plot(t, setpoints, "--", color=color, label=f"setpoint {w}")
            targets = [r.get(f"target_{w}") for r in rows]
            if (
                setpoint_col != f"target_{w}"
                and f"target_{w}" in header
                and any(
                    a is not None and b is not None and not math.isclose(a, b)
                    for a, b in zip(setpoints, targets)
                )
            ):
                axes[0].plot(
                    t, targets, ":", color=color, alpha=0.6, label=f"target {w}"
                )
        axes[0].plot(
            t, [r[f"speed_{w}"] for r in rows], color=color, label=f"speed {w}"
        )
        axes[1].plot(t, duties, color=color, label=f"duty {w}")
    axes[0].set_ylabel("speed (rev/s)")
    axes[1].set_ylabel("duty")
    if has_distance:
        ax = axes[2]
        # "inf" = nothing in range; leave a gap in the line there
        dist = [
            r["distance_cm"]
            if r.get("distance_cm") is not None and math.isfinite(r["distance_cm"])
            else float("nan")
            for r in rows
        ]
        ax.plot(t, dist, color="k", label="obstacle")
        if "threshold_cm" in header:
            ax.plot(
                t,
                [r.get("threshold_cm") for r in rows],
                "--",
                color="C3",
                label="brake threshold",
            )
        ax.set_ylabel("distance (cm)")
        ax.set_ylim(bottom=0)
        ax.legend()
        # mark each brake with a line on every panel
        for prev, row in zip(rows, rows[1:]):
            if row.get("brake") and not prev.get("brake"):
                for a in axes:
                    a.axvline(row["t"], color="C3", alpha=0.4, lw=1)
    if has_battery:
        axes[-1].plot(t, [r.get("battery_v") for r in rows], color="k")
        axes[-1].set_ylabel("battery (V)")
    axes[-1].set_xlabel("time (s)")
    axes[0].set_title(title)
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_sweep(header, rows, path, title):
    fig, ax = plt.subplots(figsize=(8, 5))
    for i, w in enumerate(wheels_in(header)):
        by_duty = {}
        for r in rows:
            if r.get("settled") and r.get(f"speed_{w}") is not None:
                by_duty.setdefault(r["duty"], []).append(r[f"speed_{w}"])
        duties = sorted(d for d, s in by_duty.items() if len(s) >= 2)
        means = [statistics.mean(by_duty[d]) for d in duties]
        stds = [statistics.stdev(by_duty[d]) for d in duties]
        ax.errorbar(
            duties,
            means,
            yerr=stds,
            fmt="o-",
            color=f"C{i}",
            capsize=3,
            label=f"wheel {w}",
        )
    ax.set_xlabel("duty")
    ax.set_ylabel("settled speed (rev/s)")
    ax.set_xlim(0, 1.05)
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_calibration(header, rows, path, title):
    """Sensor voltage against distance, from distance_calibrate.py."""
    rows = sorted(rows, key=lambda r: r["distance_cm"])
    d = [r["distance_cm"] for r in rows]
    v = [r["voltage"] for r in rows]
    err = [r.get("voltage_std") or 0 for r in rows]
    peak = max(range(len(v)), key=v.__getitem__)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.errorbar(
        d[peak:],
        v[peak:],
        yerr=err[peak:],
        fmt="o-",
        color="C0",
        capsize=3,
        label="usable: voltage falls with distance",
    )
    if peak:
        ax.errorbar(
            d[: peak + 1],
            v[: peak + 1],
            yerr=err[: peak + 1],
            fmt="o--",
            color="0.6",
            capsize=3,
            label="inside minimum range",
        )
    ax.set_xlabel("distance from rover front (cm)")
    ax.set_ylabel("sensor voltage (V)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def classify(header, rows):
    """'sweep', 'run' or 'cal', or raises ValueError for CSVs this can't plot."""
    if rows and "distance_cm" in header and "voltage" in header and "t" not in header:
        return "cal"
    if not rows or not wheels_in(header):
        raise ValueError("no speed columns, not a robot log")
    if "duty" in header and "settled" in header:
        return "sweep"
    if "t" in header:
        return "run"
    raise ValueError("unrecognised column layout")


def unique_path(path):
    """path, or path with -1, -2... added if that name is taken."""
    candidate, n = path, 1
    while candidate.exists():
        candidate = path.with_name(f"{path.stem}-{n}{path.suffix}")
        n += 1
    return candidate


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("logs", type=Path, help="folder with the CSV logs")
    p.add_argument("new_graphs", type=Path, help="where the new graphs go")
    p.add_argument("old_graphs", type=Path, help="where earlier graphs are moved to")
    p.add_argument("--force", action="store_true", help="replot everything")
    args = p.parse_args()

    args.new_graphs.mkdir(parents=True, exist_ok=True)
    args.old_graphs.mkdir(parents=True, exist_ok=True)
    manifest_path = args.old_graphs / ".plotted.json"
    manifest = (
        {}
        if args.force or not manifest_path.exists()
        else json.loads(manifest_path.read_text())
    )

    todo = []
    for csv_path in sorted(args.logs.glob("*.csv")):
        digest = hashlib.sha1(csv_path.read_bytes()).hexdigest()
        if digest in manifest:
            continue
        try:
            header, rows = read_csv(csv_path)
            kind = classify(header, rows)
        except (ValueError, KeyError, csv.Error, UnicodeDecodeError) as err:
            print(f"skipped {csv_path.name}: {err}")
            manifest[digest] = {"csv": csv_path.name, "graph": None}  # don't retry it
            continue
        todo.append((csv_path, digest, kind, header, rows))
    manifest_path.write_text(json.dumps(manifest, indent=1))
    if not todo:
        print("nothing new to plot")
        return

    moved = 0
    for old in sorted(args.new_graphs.glob("*.png")):
        shutil.move(str(old), unique_path(args.old_graphs / old.name))
        moved += 1
    if moved:
        print(f"moved {moved} earlier graph(s) to {args.old_graphs}")

    plotted = 0
    for csv_path, digest, kind, header, rows in todo:
        png_path = unique_path(args.new_graphs / (csv_path.stem + ".png"))
        try:
            plot = {"sweep": plot_sweep, "cal": plot_calibration}.get(
                kind, plot_timeseries
            )
            plot(header, rows, png_path, csv_path.stem)
        except (ValueError, KeyError, TypeError) as err:
            print(f"failed {csv_path.name}: {err}")
            continue
        manifest[digest] = {"csv": csv_path.name, "graph": png_path.name}
        manifest_path.write_text(json.dumps(manifest, indent=1))  # save as we go
        plotted += 1
        print(f"{kind:5} {png_path.name}")
    print(f"\nplotted {plotted} new graph(s) into {args.new_graphs}")


if __name__ == "__main__":
    main()
