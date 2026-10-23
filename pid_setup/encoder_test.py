#!/usr/bin/env python3
"""Lab 7, activity 1: odometer and speedometer from the wheel encoders.

python3 encoder_test.py          # spin the wheels by hand
python3 encoder_test.py --drive  # also run both motors at 40% (wheels off the ground!)
"""

import argparse
import math
import time

from gpiozero import Device, Motor, RotaryEncoder

# ---- Change to match your wiring (BCM numbers) ----
ENC_A_PINS = (19, 20)  # encoder on motor_a: phase A, phase B
ENC_B_PINS = (5, 26)  # encoder on motor_b: phase A, phase B
COUNTS_PER_REV = None  # steps for one full WHEEL turn; measure it, then fill in
WHEEL_DIAMETER_M = 0.065
SAMPLE_S = 0.2

try:  # same backend as your motor code, if pigpiod is running
    import pigpio
    from gpiozero.pins.pigpio import PiGPIOFactory

    # pigpio bug: its notification thread can close its own socket while
    # stop() is still trying to message it, giving "OSError: [Errno 9] Bad
    # file descriptor" at exit. A closed socket there means the thread has
    # already stopped, which is all stop() wants, so ignore that error.
    _pigpio_notify_stop = pigpio._callback_thread.stop

    def _notify_stop(self):
        try:
            _pigpio_notify_stop(self)
        except OSError:
            pass

    pigpio._callback_thread.stop = _notify_stop
    Device.pin_factory = PiGPIOFactory()
except Exception:
    print("pigpiod not available, using the default pin factory")

# max_steps=0 matters: the default (16) clamps .steps at +/-16
enc_a = RotaryEncoder(*ENC_A_PINS, max_steps=0)
enc_b = RotaryEncoder(*ENC_B_PINS, max_steps=0)


def describe(steps, steps_per_s):
    text = f"{steps:7d} steps {steps_per_s:8.1f} steps/s"
    if COUNTS_PER_REV:
        revs = steps / COUNTS_PER_REV
        dist = revs * math.pi * WHEEL_DIAMETER_M
        speed = steps_per_s / COUNTS_PER_REV * math.pi * WHEEL_DIAMETER_M
        text += f" | {dist:6.2f} m {speed:5.2f} m/s"
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", action="store_true", help="run motors at 40%%")
    args = parser.parse_args()

    motors = []
    if args.drive:
        motors = [Motor(forward=12, backward=18), Motor(forward=13, backward=25)]
        for m in motors:
            m.forward(0.4)

    last_a, last_b, last_t = enc_a.steps, enc_b.steps, time.monotonic()
    try:
        while True:
            time.sleep(SAMPLE_S)
            now = time.monotonic()
            a, b = enc_a.steps, enc_b.steps
            dt = now - last_t
            print(
                f"A: {describe(a, (a - last_a) / dt)}   "
                f"B: {describe(b, (b - last_b) / dt)}"
            )
            last_a, last_b, last_t = a, b, now
    except KeyboardInterrupt:
        pass
    finally:
        for m in motors:
            m.stop()
        enc_a.close()
        enc_b.close()
        for m in motors:
            m.close()


if __name__ == "__main__":
    main()
