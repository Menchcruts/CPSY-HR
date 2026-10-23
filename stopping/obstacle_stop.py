#!/usr/bin/env python3
"""Drive straight with the speed controller, and brake before hitting something.

    python3 obstacle_stop.py                          # 2 rev/s, keep a 25 cm gap
    python3 obstacle_stop.py --setpoint 3 --stop-cm 30
    python3 obstacle_stop.py --resume --duration 60   # stop, wait until clear, drive on

Every loop (50 ms, the period the gains were tuned at) it reads the distance
sensor and works out how much room the rover needs to stop at its current
speed v (m/s, from the encoders):

    threshold = stop_cm + v * latency + v^2 / (2 * decel)

When the obstacle is at or inside the threshold, both motors brake: both
DRV8833 inputs high, which shorts the motor and stops it much faster than
coasting. After each stop it prints how far the encoders say the rover rolled
after braking and the deceleration that implies, so --decel can come from
your own measurements.

Needs distance_table.csv from distance_calibrate.py, pigpiod, numpy,
simple-pid, and encoders.py and distance_sensor.py in the same folder.
"""
import argparse
import csv
import math
import time
from pathlib import Path

from gpiozero import Device, Motor
from simple_pid import PID

from distance_sensor import DistanceSensor
from encoders import EncoderReader

# ---- Your robot (BCM numbers) ----
MOTOR_PINS = {"a": (12, 18), "b": (13, 25)}   # forward, backward
ENCODER_PINS = {"a": (19, 20), "b": (5, 26)}  # phase A, phase B (b reversed: mirrored motor)
COUNTS_PER_REV = 690
WHEEL_DIAMETER_M = 0.045   # every distance and speed below scales with this
SENSOR_CHANNEL = 0         # MCP3008 channel of the GP2Y0A02YK (rated 20-150 cm)
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

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


def ramp_toward(current, target, max_change):
    if current < target:
        return min(current + max_change, target)
    return max(current - max_change, target)


class Wheel:
    def __init__(self, name, encoder, args):
        self.name = name
        self.motor = Motor(*MOTOR_PINS[name])
        self.encoder = encoder
        self.pid = PID(args.kp, args.ki, args.kd, setpoint=0,
                       sample_time=None, output_limits=(0, 1))   # forwards only
        self.setpoint = 0.0
        self.duty = 0.0
        self.braking = False
        self.last_steps = encoder.steps
        self.speed = 0.0
        self.saturated_since = None

    def measure(self, dt):
        steps = self.encoder.steps
        self.speed = (steps - self.last_steps) / dt / COUNTS_PER_REV
        self.last_steps = steps

    def drive(self, target, max_change, now):
        self.braking = False
        self.setpoint = ramp_toward(self.setpoint, target, max_change)
        self.pid.setpoint = self.setpoint
        self.duty = self.pid(self.speed)
        self._check_runaway(now)
        self.motor.value = self.duty

    def brake(self):
        """Both inputs high: the driver shorts the motor, which brakes it hard."""
        self.braking = True
        self.setpoint = self.duty = 0.0
        self.pid.reset()          # a fresh start (from rest) when driving again
        self.motor.forward_device.value = 1
        self.motor.backward_device.value = 1

    def _check_runaway(self, now):
        stuck = self.duty > 0.99 and self.speed < 0.1 * self.setpoint
        if not stuck:
            self.saturated_since = None
        elif self.saturated_since is None:
            self.saturated_since = now
        elif now - self.saturated_since > 1.0:
            raise RuntimeError(
                f"wheel {self.name}: full power but speed is {self.speed:.2f} rev/s. "
                "Stuck, or the encoder is unplugged?")

    def close(self):
        self.motor.stop()
        self.motor.close()


def stopping_cm(v, args):
    """Distance travelled while reacting, plus while braking, at v m/s."""
    return 100 * (v * args.latency + v * v / (2 * args.decel))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--setpoint", type=float, default=2.0, help="cruise speed in rev/s (default 2)")
    p.add_argument("--kp", type=float, default=0.4)
    p.add_argument("--ki", type=float, default=2.325)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--ramp", type=float, default=5.0,
                   help="speed-up rate in rev/s per second (default 5, 0 = jump)")
    p.add_argument("--stop-cm", type=float, default=25.0,
                   help="gap to keep from obstacles, in cm (default 25)")
    p.add_argument("--decel", type=float, default=1.0,
                   help="braking deceleration in m/s^2 (default 1, cautious; measure it)")
    # The GP2Y0A02YK updates every 38 +/- 10 ms, the median filter adds about one
    # loop, and the loop itself up to 50 ms: about 0.15 s in the worst case.
    p.add_argument("--latency", type=float, default=0.15,
                   help="seconds from obstacle to brake: sensor, filter, loop (default 0.15)")
    p.add_argument("--resume", action="store_true",
                   help="after a stop, drive on once the path is clear (default: end the run)")
    p.add_argument("--clear-cm", type=float, default=15.0,
                   help="with --resume: path must be this much beyond --stop-cm for 1 s")
    p.add_argument("--duration", type=float, default=15.0, help="seconds (default 15)")
    p.add_argument("--sample", type=float, default=0.05,
                   help="loop period in seconds (default 0.05; the gains were tuned at it)")
    p.add_argument("--prefix", help="label prepended to the log name")
    args = p.parse_args()
    if args.setpoint <= 0 or args.decel <= 0 or args.latency < 0:
        p.error("--setpoint and --decel must be positive, --latency not negative")
    return args


def main():
    args = parse_args()
    sensor = DistanceSensor(channel=SENSOR_CHANNEL)
    near, far = sensor.convert.min_cm, sensor.convert.max_cm
    cruise_v = args.setpoint * WHEEL_CIRCUMFERENCE_M
    needed = args.stop_cm + stopping_cm(cruise_v, args)
    print(f"sensor calibrated for {near:g}-{far:g} cm; wheel diameter {WHEEL_DIAMETER_M * 100:g} cm")
    print(f"cruise {args.setpoint:g} rev/s = {cruise_v:.2f} m/s: brakes when an obstacle is "
          f"within {needed:.0f} cm at full speed")
    if args.stop_cm < near:
        sensor.close()
        raise SystemExit(f"--stop-cm must be at least {near:g}: closer than the calibrated range, "
                         "the sensor can read an obstacle as far away")
    if needed > far:
        sensor.close()
        raise SystemExit(f"at {args.setpoint:g} rev/s the rover needs {needed:.0f} cm to stop, "
                         f"but the sensor only sees {far:g} cm: lower --setpoint")

    reader = EncoderReader(Device.pin_factory.connection, list(ENCODER_PINS.values()))
    wheels = [Wheel(n, enc, args) for n, enc in zip(ENCODER_PINS, reader.encoders)]
    max_change = args.ramp * args.sample if args.ramp > 0 else math.inf

    first = sensor.distance()
    state = "drive"
    if first <= args.stop_cm + args.clear_cm:
        if not args.resume:
            sensor.close()
            reader.close()
            for w in wheels:
                w.close()
            raise SystemExit(f"path blocked: obstacle at {first:.0f} cm")
        print(f"path blocked ({first:.0f} cm), waiting for it to clear")
        state = "stopped"
        for w in wheels:
            w.brake()

    rows, stops = [], []
    event = None            # the stop in progress
    still_since = clear_since = stopped_at = None
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
            dt, last = now - last, now
            reader.update()
            for w in wheels:
                w.measure(dt)
            dist = sensor.distance()
            v = max(0.0, sum(w.speed for w in wheels) / len(wheels)) * WHEEL_CIRCUMFERENCE_M
            threshold = args.stop_cm + stopping_cm(v, args)

            if state == "drive":
                if dist <= threshold:
                    for w in wheels:
                        w.brake()
                    state = "brake"
                    event = {"t": t, "dist": dist, "threshold": threshold, "v": v,
                             "steps": [w.encoder.steps for w in wheels]}
                    print(f"BRAKE at t={t:.2f} s: obstacle {dist:.1f} cm, "
                          f"threshold {threshold:.1f} cm, speed {v:.2f} m/s")
                else:
                    for w in wheels:
                        w.drive(args.setpoint, max_change, now)
            elif state == "brake":
                still = all(abs(w.speed) < 0.05 for w in wheels)
                still_since = (still_since or now) if still else None
                if (still_since and now - still_since >= 0.1) or t - event["t"] > 1.5:
                    revs = sum(w.encoder.steps - s for w, s in zip(wheels, event["steps"]))
                    roll_cm = revs / len(wheels) / COUNTS_PER_REV * WHEEL_CIRCUMFERENCE_M * 100
                    event.update(roll_cm=roll_cm, stop_time=t - event["t"], after=dist)
                    decel = event["v"] ** 2 / (2 * roll_cm / 100) if roll_cm > 0.5 else None
                    print(f"  stopped after rolling {roll_cm:.1f} cm in {event['stop_time']:.2f} s"
                          + (f" (braking at {decel:.2f} m/s^2)" if decel else ""))
                    # Had the reading been up to date, the gap now would be the gap at
                    # the brake minus the roll; anything closer was travelled meanwhile.
                    lag_cm = event["dist"] - roll_cm - dist
                    line = f"  sensor now reads {dist:.1f} cm"
                    if event["v"] > 0.2 and math.isfinite(lag_cm):
                        line += (f", {lag_cm:.1f} cm closer than {event['dist']:.1f} - "
                                 f"{roll_cm:.1f}: the reading lagged ~{lag_cm / 100 / event['v']:.2f} s")
                    print(line)
                    stops.append(event)
                    state, stopped_at, clear_since = "stopped", now, None
            elif state == "stopped":
                if not args.resume:
                    if stopped_at is not None and now - stopped_at >= 1.0:
                        break       # a second of stillness logged after the stop
                else:
                    clear = dist > args.stop_cm + args.clear_cm
                    clear_since = (clear_since or now) if clear else None
                    if clear_since and now - clear_since >= 1.0:
                        print(f"path clear ({dist:.0f} cm) at t={t:.2f} s, driving on")
                        state = "drive"

            row = {"t": round(t, 3), "state": state, "brake": int(state != "drive"),
                   "distance_cm": round(dist, 1), "threshold_cm": round(threshold, 1),
                   "ground_speed_ms": round(v, 3)}
            for w in wheels:
                row[f"target_{w.name}"] = args.setpoint
                row[f"setpoint_{w.name}"] = round(w.setpoint, 3)
                row[f"speed_{w.name}"] = round(w.speed, 3)
                row[f"duty_{w.name}"] = round(w.duty, 3)
                row[f"steps_{w.name}"] = round(w.encoder.steps, 2)
            rows.append(row)
            if len(rows) % max(1, round(0.25 / args.sample)) == 0:
                print(f"t={t:5.2f} {state:7} obstacle {dist:6.1f} cm (brake at {threshold:5.1f})  "
                      + "  ".join(f"{w.name}: {w.speed:4.2f} rev/s" for w in wheels))
    except KeyboardInterrupt:
        print("stopped by Ctrl+C")
    except RuntimeError as err:
        print(f"ABORTED: {err}")
    finally:
        for w in wheels:
            w.brake()
        time.sleep(0.5)          # hold the brake until the rover has stopped
        for w in wheels:
            w.close()
        reader.close()
        sensor.close()

    if not rows:
        return
    Path("logs").mkdir(exist_ok=True)
    prefix = f"{args.prefix}_" if args.prefix else ""
    name = (f"{prefix}{time.strftime('%Y%m%d-%H%M%S')}_obstacle_sp{args.setpoint:g}"
            f"_stop{args.stop_cm:g}_dec{args.decel:g}")
    path = Path("logs") / f"{name}.csv"
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"log: {path}")
    for i, s in enumerate(stops, 1):
        print(f"stop {i}: braked at {s['dist']:.1f} cm from {s['v']:.2f} m/s, "
              f"rolled {s['roll_cm']:.1f} cm, sensor then read {s['after']:.1f} cm")


if __name__ == "__main__":
    main()
