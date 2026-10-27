"""Shared rover code: pins, the pigpio connection, and a speed-controlled wheel.

    import rover
    pi = rover.connect()                       # pigpio backend for gpiozero
    reader = EncoderReader(pi, list(rover.ENCODER_PINS.values()))
    wheels = {n: rover.Wheel(n, enc, kp=0.4, ki=2.325)
              for n, enc in zip(rover.ENCODER_PINS, reader.encoders)}
    # every loop:
    reader.update()
    for w in wheels.values():
        w.measure(dt)
        w.drive(setpoint_rev_s, now)          # or w.brake()

Each Wheel is the speed controller tuned in Lab 7: a PI controller on the
encoder speed, with the direction lock (forwards-only output for a positive
setpoint, backwards-only for a negative one, so it slows down by coasting).
"""

import math

from gpiozero import Device, Motor
from simple_pid import PID

# ---- Your robot (BCM numbers) ----
MOTOR_PINS = {"a": (12, 18), "b": (13, 25)}  # forward, backward
ENCODER_PINS = {
    "a": (19, 20),
    "b": (5, 26),
}  # phase A, phase B (b reversed: mirrored motor)
COUNTS_PER_REV = 690
WHEEL_DIAMETER_M = 0.045
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
LEFT_WHEEL = (
    "b"  # CHECK: the wheel on the rover's left, looking in the driving direction
)

# The speed gains tuned in Lab 7 (floor, 50 ms loop)
SPEED_KP, SPEED_KI, SPEED_KD = 0.4, 2.325, 0.0


def connect():
    """Make gpiozero use pigpio and return the pigpio connection."""
    try:
        import pigpio
        from gpiozero.pins.pigpio import PiGPIOFactory
    except ImportError:
        raise SystemExit("pigpio isn't installed")

    # pigpio bug: its notification thread can close its own socket while
    # stop() still writes to it ("OSError: [Errno 9] Bad file descriptor" at
    # exit). A closed socket there means the thread has already stopped.
    original_stop = pigpio._callback_thread.stop

    def stop(self):
        try:
            original_stop(self)
        except OSError:
            pass

    pigpio._callback_thread.stop = stop
    try:
        Device.pin_factory = PiGPIOFactory()
    except Exception:
        raise SystemExit(
            "Can't connect to pigpiod. Start it with: sudo systemctl start pigpiod"
        )
    return Device.pin_factory.connection


def ramp_toward(current, target, max_change):
    """Move current toward target by at most max_change."""
    if current < target:
        return min(current + max_change, target)
    return max(current - max_change, target)


def direction_limits(setpoint):
    """Direction lock: forwards only, backwards only, or stopped."""
    if setpoint > 0:
        return (0, 1)
    if setpoint < 0:
        return (-1, 0)
    return (0, 0)


class Wheel:
    """One motor + encoder under speed control (rev/s)."""

    def __init__(self, name, encoder, kp=SPEED_KP, ki=SPEED_KI, kd=SPEED_KD):
        self.name = name
        self.motor = Motor(*MOTOR_PINS[name])
        self.encoder = encoder
        self.pid = PID(kp, ki, kd, setpoint=0, sample_time=None, output_limits=(0, 0))
        self.setpoint = 0.0
        self.duty = 0.0
        self.speed = 0.0
        self.last_steps = encoder.steps
        self.saturated_since = None

    def measure(self, dt):
        """Update .speed (rev/s) from the encoder; call once per loop."""
        steps = self.encoder.steps
        self.speed = (steps - self.last_steps) / dt / COUNTS_PER_REV
        self.last_steps = steps

    def drive(self, setpoint, now):
        """Run the speed controller one step toward setpoint (rev/s)."""
        self.setpoint = setpoint
        limits = direction_limits(setpoint)
        if limits != self.pid.output_limits:
            self.pid.output_limits = limits  # also clamps the integral to the new range
        self.pid.setpoint = setpoint
        self.duty = self.pid(self.speed)
        self._check_runaway(now)
        self.motor.value = self.duty

    def brake(self):
        """Both driver inputs high: the motor is shorted, which brakes it hard."""
        self.setpoint = self.duty = 0.0
        self.pid.reset()
        self.pid.output_limits = (0, 0)
        self.motor.forward_device.value = 1
        self.motor.backward_device.value = 1

    def _check_runaway(self, now):
        # Full power for a second with the wheel barely moving the right way
        # usually means a stalled wheel or an unplugged encoder.
        direction = 1 if self.setpoint >= 0 else -1
        stuck = abs(self.duty) > 0.99 and self.speed * direction < 0.1 * abs(
            self.setpoint
        )
        if not stuck:
            self.saturated_since = None
        elif self.saturated_since is None:
            self.saturated_since = now
        elif now - self.saturated_since > 1.0:
            raise RuntimeError(
                f"wheel {self.name}: full power but speed is {self.speed:.2f} rev/s. "
                "Stalled, or the encoder is unplugged?"
            )

    def close(self):
        self.motor.stop()
        self.motor.close()
