#!/usr/bin/env python3
"""Lab 7, activity 2: speed-control test for one or both wheels.

Lift the wheels off the ground for the first runs.

    python3 pid_motor_test.py a --duty 0.5                        # open loop, no PID
    python3 pid_motor_test.py a --setpoint 2 --kp 0.2             # P only
    python3 pid_motor_test.py a --setpoint 2 --kp 0.2 --ki 0.4    # PI
    python3 pid_motor_test.py a --setpoint 2 --kp 0.2 --ki 0.4 --step2 1
    python3 pid_motor_test.py both --setpoint 2 --kp 0.2 --ki 0.4
    python3 pid_motor_test.py both --setpoint 2 --setpoint-b 1 --kp 0.2 --ki 0.4   # turn

Every run writes a CSV to ./logs/, named by timestamp, motor and gains. Plot
them on your desktop with plot_logs.py, or add --plot to plot here into ./graphs/.

Needs pigpiod running, numpy, simple-pid, and encoders.py in the same folder.
"""
import argparse
import csv
import statistics
import time
from pathlib import Path

import numpy as np
from gpiozero import Device, Motor
from simple_pid import PID

from encoders import EncoderReader

# ---- Your robot (BCM numbers) ----
MOTOR_PINS = {"a": (12, 18), "b": (13, 25)}   # forward, backward
ENCODER_PINS = {"a": (19, 20), "b": (5, 26)}  # phase A, phase B (b reversed: mirrored motor)
COUNTS_PER_REV = 690    # from your activity 1 calibration; None = speeds in steps/s
DIVIDER_RATIO = 3.0     # (R1 + R2) / R2 of your battery voltage divider
UNIT = "rev/s" if COUNTS_PER_REV else "steps/s"

try:  # pigpio backend, with the fix for the EBADF error at exit
    import pigpio
    from gpiozero.pins.pigpio import PiGPIOFactory

    _pigpio_notify_stop = pigpio._callback_thread.stop

    def _notify_stop(self):
        try:
            _pigpio_notify_stop(self)
        except OSError:
            pass  # notification thread already stopped and closed its socket

    pigpio._callback_thread.stop = _notify_stop
    Device.pin_factory = PiGPIOFactory()
except Exception:
    raise SystemExit("Can't connect to pigpiod. Start it with: sudo systemctl start pigpiod")


class Wheel:
    def __init__(self, name, encoder, args):
        self.name = name
        self.motor = Motor(*MOTOR_PINS[name])
        self.encoder = encoder
        self.pid = None
        if args.duty is None:
            limits = (-1, 1) if args.allow_reverse else (0, 1)
            self.pid = PID(args.kp, args.ki, args.kd, setpoint=args.setpoint,
                           sample_time=None, output_limits=limits)
        self.last_steps = self.encoder.steps
        self.speed = 0.0
        self.saturated_since = None

    def update(self, dt, target, alpha, now):
        steps = self.encoder.steps
        raw = (steps - self.last_steps) / dt / (COUNTS_PER_REV or 1)
        self.last_steps = steps
        self.speed = alpha * raw + (1 - alpha) * self.speed  # alpha=1: no filter
        if self.pid is None:
            duty = target
        else:
            self.pid.setpoint = target
            duty = self.pid(self.speed)
            self._check_runaway(duty, target, now)
        self.motor.value = duty
        return steps, duty

    def _check_runaway(self, duty, target, now):
        # Full power for a full second with the wheel barely moving the right
        # way usually means an unplugged encoder or one counting backwards.
        direction = 1 if target >= 0 else -1
        stuck = abs(duty) > 0.99 and self.speed * direction < 0.1 * abs(target)
        if not stuck:
            self.saturated_since = None
        elif self.saturated_since is None:
            self.saturated_since = now
        elif now - self.saturated_since > 1.0:
            raise RuntimeError(
                f"wheel {self.name}: full power but speed is {self.speed:.2f} {UNIT}. "
                "Encoder unplugged, or counting backwards (swap its pin order)?")

    def close(self):
        self.motor.stop()
        self.motor.close()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("motor", choices=["a", "b", "both"])
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--setpoint", type=float, help=f"target speed in {UNIT} (PID mode)")
    mode.add_argument("--duty", type=float, help="fixed duty -1..1 (open loop, no PID)")
    p.add_argument("--kp", type=float, default=0.0)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--setpoint-b", type=float,
                   help="different target for wheel b with 'both' (turning); "
                        "negative spins it backwards (needs --allow-reverse)")
    p.add_argument("--step2", type=float,
                   help="switch the target to this value halfway through the run")
    p.add_argument("--duration", type=float, default=6.0, help="seconds (default 6)")
    p.add_argument("--sample", type=float, default=0.05,
                   help="control loop period in seconds (default 0.05)")
    p.add_argument("--alpha", type=float, default=1.0,
                   help="speed filter 0..1; lower = smoother but laggier (default 1, off)")
    p.add_argument("--allow-reverse", action="store_true",
                   help="let the PID drive the motor backwards (output -1..1)")
    p.add_argument("--battery-ch", type=int,
                   help="MCP3008 channel of the battery voltage divider")
    p.add_argument("--prefix", help="label prepended to the log name, e.g. kp_floor")
    p.add_argument("--plot", action="store_true",
                   help="also plot on the Pi (slow); otherwise use plot_logs.py on your desktop")
    args = p.parse_args()
    if args.setpoint_b is not None and (args.motor != "both" or args.setpoint is None
                                        or args.step2 is not None):
        p.error("--setpoint-b needs 'both' and --setpoint, and can't be used with --step2")
    targets = [x for x in (args.setpoint, args.setpoint_b, args.step2) if x is not None]
    if args.duty is None and min(targets) < 0 and not args.allow_reverse:
        p.error("negative targets need --allow-reverse")
    return args


def log_name(args):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    prefix = ""
    if args.prefix is not None:
        prefix = f"{args.prefix}_"
    if args.duty is not None:
        return f"{prefix}{stamp}_open_{args.motor}_duty{args.duty:g}"
    spb = "" if args.setpoint_b is None else f"_spb{args.setpoint_b:g}"
    return (f"{prefix}{stamp}_pid_{args.motor}_sp{args.setpoint:g}{spb}_kp{args.kp:g}"
            f"_ki{args.ki:g}_kd{args.kd:g}")


def oscillation_period(speeds, sample):
    """Period of a regular oscillation in the speed, or None if it's just noise.

    Uses autocorrelation: random sensor noise doesn't line up with a shifted copy
    of itself, but a real oscillation does at a shift of one period.
    """
    x = np.asarray(speeds) - np.mean(speeds)
    ac = np.correlate(x, x, "full")[len(x) - 1:]
    if ac[0] == 0:
        return None
    ac = ac / ac[0]
    for lag in range(2, len(x) // 2):
        if ac[lag] > 0.5 and ac[lag] >= ac[lag - 1] and ac[lag] >= ac[lag + 1]:
            return lag * sample
    return None


def summarize(rows, names, args, reader):
    """Per wheel and per target: settled mean/std, error, overshoot, oscillation."""
    print(f"encoder check: {reader.dropped} dropped reports, "
          f"{reader.invalid} invalid transitions")
    if reader.dropped or reader.invalid:
        print("  WARNING: counts were lost, so the speeds in this run read too low")
    for i, name in enumerate(names):
        tcol, scol = 1 + 4 * i, 2 + 4 * i
        segments, first = [], 0
        for j in range(1, len(rows) + 1):
            if j == len(rows) or rows[j][tcol] != rows[first][tcol]:
                segments.append(rows[first:j])
                first = j
        for k, seg in enumerate(segments):
            target = seg[0][tcol]
            t0, t1 = seg[0][0], seg[-1][0]
            settled = [r for r in seg if r[0] >= t0 + 0.6 * (t1 - t0)]
            if len(settled) < 2:
                continue
            speeds = [r[scol] for r in settled]
            mean, std = statistics.mean(speeds), statistics.stdev(speeds)
            line = (f"wheel {name}, target {target:g}: settled speed {mean:.2f} "
                    f"± {std:.2f} {UNIT}")
            if args.duty is None:
                line += f", error {mean - target:+.2f}"
                if k == 0 and target != 0:
                    sign = 1 if target > 0 else -1
                    peak = max(r[scol] * sign for r in seg)
                    line += f", overshoot {max(0, (peak - abs(target)) / abs(target) * 100):.0f}%"
                period = oscillation_period(speeds, args.sample) if std > 0.05 else None
                if period:
                    line += f", possible oscillation (period ~{period:.2f} s, check the plot)"
            print(line)


def plot(rows, names, args, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plot")
        return
    t = [r[0] for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    for i, name in enumerate(names):
        color = f"C{i}"
        if args.duty is None:
            ax1.plot(t, [r[1 + 4 * i] for r in rows], "--", color=color,
                     label=f"setpoint {name}")
        ax1.plot(t, [r[2 + 4 * i] for r in rows], color=color, label=f"speed {name}")
        ax2.plot(t, [r[3 + 4 * i] for r in rows], color=color, label=f"duty {name}")
    ax1.set_ylabel(f"speed ({UNIT})")
    ax2.set_ylabel("duty")
    ax2.set_xlabel("time (s)")
    ax1.set_title(path.stem)
    ax1.legend()
    ax2.legend()
    fig.tight_layout()
    path.parent.mkdir(exist_ok=True)
    fig.savefig(path)
    print(f"plot: {path}")


def main():
    args = parse_args()
    names = ["a", "b"] if args.motor == "both" else [args.motor]

    read_battery = lambda: ""
    if args.battery_ch is not None:
        from gpiozero import MCP3008
        adc = MCP3008(channel=args.battery_ch)
        read_battery = lambda: round(adc.value * 3.3 * DIVIDER_RATIO, 2)
        volts = read_battery()
        print(f"battery: {volts} V" + ("  (below 6 V: USB only?)" if volts < 6 else ""))

    reader = EncoderReader(Device.pin_factory.connection,
                           [ENCODER_PINS[n] for n in names])
    wheels = [Wheel(n, enc, args) for n, enc in zip(names, reader.encoders)]
    first = args.setpoint if args.duty is None else args.duty
    rows = []
    try:
        start = last = time.monotonic()
        next_t = start + args.sample
        while True:
            time.sleep(max(0.0, next_t - time.monotonic()))
            next_t += args.sample
            now = time.monotonic()
            t = now - start
            if t >= args.duration:
                break
            second_half = args.step2 is not None and t >= args.duration / 2
            target = args.step2 if second_half else first
            dt, last = now - last, now
            reader.update()
            row = [round(t, 3)]
            for w in wheels:
                w_target = args.setpoint_b if (w.name == "b" and args.setpoint_b
                                               is not None) else target
                steps, duty = w.update(dt, w_target, args.alpha, now)
                row += [w_target, round(w.speed, 3), round(duty, 3), round(steps, 2)]
            row.append(read_battery())
            rows.append(row)
            if len(rows) % max(1, round(0.25 / args.sample)) == 0:
                print(f"t={t:5.2f}  " + "  ".join(
                    f"{w.name}: {row[2 + 4 * i]:6.2f}/{row[1 + 4 * i]:g} {UNIT} "
                    f"duty {row[3 + 4 * i]:+.2f}" for i, w in enumerate(wheels)))
    except KeyboardInterrupt:
        print("stopped")
    except RuntimeError as err:
        print(f"ABORTED: {err}")
    finally:
        for w in wheels:
            w.close()
        reader.update()  # count the last edges before closing, for the summary
        reader.close()

    if not rows:
        return
    Path("logs").mkdir(exist_ok=True)
    path = Path("logs") / (log_name(args) + ".csv")
    header = ["t"]
    for n in names:
        header += [f"target_{n}", f"speed_{n}", f"duty_{n}", f"steps_{n}"]
    header.append("battery_v")
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"log: {path}")
    summarize(rows, names, args, reader)
    if args.plot:
        plot(rows, names, args, Path("graphs") / (path.stem + ".png"))


if __name__ == "__main__":
    main()
