#!/usr/bin/env python3
"""Follow the edge of a black line on a white floor, with one TCS34725.

    python3 line_follow.py                      # follow the line's left edge at 1.5 rev/s
    python3 line_follow.py --edge right --speed 2 --line-kp 2.5

Start with the sensor over the edge you want to follow (blackness ~0.5).

How it works: a single sensor can't tell where the line is, only how much of
it is under the sensor. So it follows one EDGE: half black, half white.
Too white means it drifted off the line, too black means it drifted onto it,
and either way it steers back. Two loops, one inside the other:

    outer (new):   blackness --line PID--> turn (rev/s)
                   left wheel = base + turn, right wheel = base - turn
    inner (Lab 7): each wheel's speed PI controller holds its setpoint

The base speed drops on sharp corners (--corner-slow), and the inner wheel
may run backwards (--min-wheel below 0) so the rover can pivot round tight
corners. If the reading stays fully white or fully black for --lost-time
seconds, the rover brakes and the run ends (line lost, or the end of the line).

The loop runs every 20 ms, not 50 ms like the Lab 7 tests: in simulation the
steering needed the shorter delay to make tight corners (the wheel gains were
tuned at 50 ms but stay stable at 20). The TCS34725 integration time must be
shorter than the loop, so calibrate it at 12 ms (the default).

Needs line_cal.json from line_sensor.py --calibrate, pigpiod, numpy,
simple-pid, and rover.py, encoders.py and line_sensor.py in the same folder.
"""

import argparse
import csv
import statistics
import time
from pathlib import Path

from simple_pid import PID

import rover
from encoders import EncoderReader
from line_sensor import LineSensor


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--edge",
        choices=["left", "right"],
        default="left",
        help="which edge of the line to follow (default left)",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=1.5,
        help="cruise wheel speed in rev/s (default 1.5)",
    )
    p.add_argument(
        "--line-kp",
        type=float,
        default=2.0,
        help="turn (rev/s) per unit of blackness error (default 2)",
    )
    p.add_argument("--line-ki", type=float, default=0.0)
    p.add_argument("--line-kd", type=float, default=0.0)
    p.add_argument(
        "--max-turn",
        type=float,
        help="largest turn command in rev/s (default: --speed)",
    )
    p.add_argument(
        "--corner-slow",
        type=float,
        default=0.7,
        help="fraction of the speed dropped at full steering error (default 0.7)",
    )
    p.add_argument(
        "--min-wheel",
        type=float,
        default=-1.5,
        help="slowest wheel setpoint, rev/s; below 0 lets the inner wheel reverse "
        "to pivot round tight corners (default -1.5; 0 = never reverse)",
    )
    p.add_argument(
        "--max-wheel",
        type=float,
        default=3.6,
        help="fastest wheel setpoint, rev/s (default 3.6: full power is ~3.8)",
    )
    p.add_argument(
        "--ramp",
        type=float,
        default=5.0,
        help="cruise speed-up rate in rev/s per second (default 5, 0 = jump)",
    )
    p.add_argument(
        "--lost-time",
        type=float,
        default=1.0,
        help="seconds of all-white or all-black before stopping (default 1)",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="seconds (default 30, -1 for forever)",
    )
    p.add_argument(
        "--sample",
        type=float,
        default=0.02,
        help="loop period in seconds (default 0.02)",
    )
    p.add_argument("--kp", type=float, default=rover.SPEED_KP, help="wheel speed gains")
    p.add_argument("--ki", type=float, default=rover.SPEED_KI)
    p.add_argument("--kd", type=float, default=rover.SPEED_KD)
    p.add_argument("--prefix", help="label prepended to the log name")
    args = p.parse_args()
    if args.max_turn is None:
        args.max_turn = args.speed
    if not 0 <= args.corner_slow < 1:
        p.error("--corner-slow must be from 0 up to (not including) 1")
    if args.speed <= 0 or args.min_wheel >= args.max_wheel:
        p.error("--speed must be positive and --min-wheel below --max-wheel")
    return args


def main():
    args = parse_args()
    pi = rover.connect()
    sensor = LineSensor(pi)
    if not sensor.cal:
        sensor.close()
        raise SystemExit("no line calibration: run python3 line_sensor.py --calibrate")
    if sensor.integration_ms / 1000 > args.sample:
        print(
            f"WARNING: sensor integration ({sensor.integration_ms} ms) is longer than the "
            "loop, so some loops reuse an old reading"
        )

    start_k = statistics.median(sensor.blackness() for _ in range(5))
    if not 0.2 <= start_k <= 0.8:
        sensor.close()
        raise SystemExit(
            f"blackness is {start_k:.2f}: place the sensor over the line's "
            f"{args.edge} edge (about 0.5) before starting"
        )

    reader = EncoderReader(pi, list(rover.ENCODER_PINS.values()))
    wheels = {
        n: rover.Wheel(n, enc, args.kp, args.ki, args.kd)
        for n, enc in zip(rover.ENCODER_PINS, reader.encoders)
    }
    right_wheel = "b" if rover.LEFT_WHEEL == "a" else "a"
    # PID output = kp * (0.5 - blackness): positive when the sensor sees too much white.
    line_pid = PID(
        args.line_kp,
        args.line_ki,
        args.line_kd,
        setpoint=0.5,
        sample_time=None,
        output_limits=(-args.max_turn, args.max_turn),
    )
    # Following the LEFT edge, the line is to the sensor's right, so "too white"
    # means steer right (left wheel faster). The right edge is the mirror image.
    side = 1 if args.edge == "left" else -1
    max_change = args.ramp * args.sample if args.ramp > 0 else float("inf")

    rows = []
    cruise = 0.0
    lost_since = None
    reason = "time up"
    try:
        start = last = time.monotonic()
        next_t = start + args.sample
        while True:
            time.sleep(max(0.0, next_t - time.monotonic()))
            next_t += args.sample
            now = time.monotonic()
            t = now - start
            if args.duration != -1 and t >= args.duration:
                break
            dt, last = now - last, now
            reader.update()
            for w in wheels.values():
                w.measure(dt)

            clear = sensor.clear()
            k = sensor.blackness(clear)
            turn = side * line_pid(k)  # > 0: steer right
            error = abs(k - 0.5) * 2  # 0 on the edge, 1 fully off it
            cruise = rover.ramp_toward(cruise, args.speed, max_change)
            base = cruise * (1 - args.corner_slow * error)
            left = min(args.max_wheel, max(args.min_wheel, base + turn))
            right = min(args.max_wheel, max(args.min_wheel, base - turn))
            wheels[rover.LEFT_WHEEL].drive(left, now)
            wheels[right_wheel].drive(right, now)

            if 0.1 < k < 0.9:
                lost_since = None
            elif lost_since is None:
                lost_since = now
            elif now - lost_since >= args.lost_time:
                reason = f"line lost (blackness {k:.2f} for {args.lost_time:g} s)"
                print(f"LOST at t={t:.2f} s: blackness {k:.2f}, braking")

            row = {
                "t": round(t, 3),
                "clear": clear,
                "blackness": round(k, 3),
                "turn": round(turn, 3),
                "base": round(base, 3),
            }
            for n, w in wheels.items():
                row[f"setpoint_{n}"] = round(w.setpoint, 3)
                row[f"speed_{n}"] = round(w.speed, 3)
                row[f"duty_{n}"] = round(w.duty, 3)
                row[f"steps_{n}"] = round(w.encoder.steps, 2)
            row["brake"] = int(reason != "time up")
            rows.append(row)
            if reason != "time up":
                break
            if len(rows) % max(1, round(0.25 / args.sample)) == 0:
                print(
                    f"t={t:5.2f}  blackness {k:4.2f}  turn {turn:+5.2f}  "
                    f"left {wheels[rover.LEFT_WHEEL].speed:4.2f}/{left:4.2f}  "
                    f"right {wheels[right_wheel].speed:4.2f}/{right:4.2f} rev/s"
                )
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    except RuntimeError as err:
        reason = f"aborted: {err}"
    finally:
        for w in wheels.values():
            w.brake()
        time.sleep(0.5)
        for w in wheels.values():
            w.close()
        reader.close()
        sensor.close()

    print(f"stopped: {reason}")
    if not rows:
        return
    Path("logs").mkdir(exist_ok=True)
    prefix = f"{args.prefix}_" if args.prefix else ""
    name = (
        f"{prefix}{time.strftime('%Y%m%d-%H%M%S')}_line_{args.edge}_sp{args.speed:g}"
        f"_lkp{args.line_kp:g}_lki{args.line_ki:g}_lkd{args.line_kd:g}"
    )
    path = Path("logs") / f"{name}.csv"
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    errors = [abs(r["blackness"] - 0.5) for r in rows]
    revs = (
        statistics.mean(r[f"steps_{n}"] for r in rows[-1:] for n in wheels)
        / rover.COUNTS_PER_REV
    )
    print(f"log: {path}")
    print(
        f"travelled ~{revs * rover.WHEEL_CIRCUMFERENCE_M:.2f} m; blackness error "
        f"mean {statistics.mean(errors):.2f}, worst {max(errors):.2f} "
        f"(0 = always on the edge, 0.5 = fully off it)"
    )


if __name__ == "__main__":
    main()
