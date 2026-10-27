#!/usr/bin/env python3
"""TCS34725 colour sensor as a line sensor, read over I2C through pigpiod.

    python3 line_sensor.py --calibrate   # measure white and black, saves line_cal.json
    python3 line_sensor.py               # live readout of brightness and blackness

The line follower only uses the sensor's "clear" channel (overall
brightness). After calibration it is turned into a blackness value:

    0 = all white under the sensor, 1 = all black, 0.5 = exactly on the edge

Mount the sensor 5-10 mm above the floor, pointing straight down, with its
LED on, and shade it from room light if you can: the reading is an average
over a spot that gets wider (and the edge blurrier) the higher it sits.
Calibrate again whenever the height, the lighting or the floor changes.
"""

import argparse
import json
import statistics
import struct
import time
from pathlib import Path

CAL_PATH = Path(__file__).with_name("line_cal.json")

ADDRESS = 0x29
CMD = 0x80  # command bit; add AUTO to read several registers in a row
AUTO = 0x20
REG_ENABLE, REG_ATIME, REG_CONTROL, REG_ID, REG_CDATAL = 0x00, 0x01, 0x0F, 0x12, 0x14
PON, AEN = 0x01, 0x02
GAINS = {1: 0, 4: 1, 16: 2, 60: 3}
CHIP_IDS = (0x44, 0x4D)  # TCS34721/5 and TCS34723/7


class LineSensor:
    def __init__(self, pi, bus=1, gain=None, integration_ms=None, cal_path=CAL_PATH):
        """gain/integration_ms default to the calibration's, or 4x and 12 ms.
        cal_path=None ignores any saved calibration (used while calibrating)."""
        self.cal = (
            json.loads(Path(cal_path).read_text())
            if cal_path and Path(cal_path).exists()
            else None
        )
        if gain is None:
            gain = self.cal["gain"] if self.cal else 4
        if integration_ms is None:
            integration_ms = self.cal["integration_ms"] if self.cal else 12
        if gain not in GAINS:
            raise ValueError(f"gain must be one of {sorted(GAINS)}")
        self.pi = pi
        self.handle = pi.i2c_open(bus, ADDRESS)
        try:
            chip = pi.i2c_read_byte_data(self.handle, CMD | REG_ID)
        except Exception:
            pi.i2c_close(self.handle)
            raise SystemExit(
                f"no TCS34725 answering at I2C address 0x{ADDRESS:02x}: "
                "check the wiring, and that I2C is enabled"
            )
        if chip not in CHIP_IDS:
            pi.i2c_close(self.handle)
            raise SystemExit(
                f"device at 0x{ADDRESS:02x} has ID 0x{chip:02x}, not a TCS34725"
            )

        cycles = max(1, min(256, round(integration_ms / 2.4)))
        self.gain = gain
        self.integration_ms = round(cycles * 2.4, 1)
        self.max_count = min(65535, cycles * 1024)
        pi.i2c_write_byte_data(self.handle, CMD | REG_ATIME, 256 - cycles)
        pi.i2c_write_byte_data(self.handle, CMD | REG_CONTROL, GAINS[gain])
        pi.i2c_write_byte_data(self.handle, CMD | REG_ENABLE, PON)
        time.sleep(0.003)  # oscillator warm-up
        pi.i2c_write_byte_data(self.handle, CMD | REG_ENABLE, PON | AEN)
        time.sleep(self.integration_ms / 1000 + 0.005)  # first full reading

        if self.cal and (self.cal["gain"], self.cal["integration_ms"]) != (
            gain,
            self.integration_ms,
        ):
            print(
                "WARNING: gain/integration differ from the calibration, so blackness is off"
            )

    def raw(self):
        """(clear, red, green, blue) counts from the last finished integration."""
        count, data = self.pi.i2c_read_i2c_block_data(
            self.handle, CMD | AUTO | REG_CDATAL, 8
        )
        if count != 8:
            raise RuntimeError(f"TCS34725 read failed ({count})")
        return struct.unpack("<4H", bytes(data))

    def clear(self):
        return self.raw()[0]

    def blackness(self, clear=None):
        """0 on white, 1 on black, 0.5 on the edge (clamped to 0..1)."""
        if not self.cal:
            raise SystemExit(
                "no line calibration: run python3 line_sensor.py --calibrate"
            )
        if clear is None:
            clear = self.clear()
        white, black = self.cal["white"], self.cal["black"]
        return min(1.0, max(0.0, (white - clear) / (white - black)))

    def close(self):
        try:
            self.pi.i2c_write_byte_data(self.handle, CMD | REG_ENABLE, 0)  # sleep
        finally:
            self.pi.i2c_close(self.handle)


def measure(sensor, seconds=1.0):
    values = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        values.append(sensor.clear())
        time.sleep(sensor.integration_ms / 1000)
    return statistics.median(values), statistics.pstdev(values)


def calibrate(sensor):
    print(
        f"gain {sensor.gain}x, integration {sensor.integration_ms} ms, "
        f"max count {sensor.max_count}\n"
    )
    input("1) Put the rover where it will drive, sensor over WHITE floor, then Enter: ")
    white, white_sd = measure(sensor)
    print(f"   white: {white:.0f} counts (± {white_sd:.0f})")
    input("2) Move it so the sensor is over the middle of the BLACK line, then Enter: ")
    black, black_sd = measure(sensor)
    print(f"   black: {black:.0f} counts (± {black_sd:.0f})")

    problems = []
    if white >= 0.95 * sensor.max_count:
        problems.append("white is saturated (at the maximum count): use a lower --gain")
    if white < 0.05 * sensor.max_count:
        problems.append(
            "white is very dim: use a higher --gain, or check the LED is on"
        )
    if white <= black:
        problems.append("white isn't brighter than black: were they swapped?")
    elif white / max(black, 1) < 1.5:
        problems.append("low contrast: lower the sensor, shade it, or check the LED")
    noise = max(white_sd, black_sd) / max(white - black, 1)
    if noise > 0.05:
        problems.append(f"noisy: readings vary by {noise:.0%} of the white-black range")
    for p in problems:
        print(f"   PROBLEM: {p}")
    if white <= black:
        print("not saved")
        return False

    CAL_PATH.write_text(
        json.dumps(
            {
                "white": white,
                "black": black,
                "gain": sensor.gain,
                "integration_ms": sensor.integration_ms,
                "calibrated": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            indent=1,
        )
    )
    sensor.cal = json.loads(CAL_PATH.read_text())
    print(f"\nsaved {CAL_PATH}: contrast {white / max(black, 1):.1f}x")
    print(
        "3) Now slide the sensor slowly across the line's edge: blackness should pass"
    )
    print(
        "   smoothly through 0.5. The width over which it goes from 0.1 to 0.9 is the"
    )
    print("   'edge zone' the steering controller works in. Ctrl+C to finish.\n")
    return True


def live(sensor):
    while True:
        c, r, g, b = sensor.raw()
        line = f"clear {c:6d}  (r {r:5d} g {g:5d} b {b:5d})"
        if sensor.cal:
            k = sensor.blackness(c)
            bar = "#" * round(k * 20)
            line += f"  blackness {k:4.2f} |{bar:<20}|"
        print(line)
        time.sleep(0.1)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--calibrate", action="store_true")
    p.add_argument(
        "--gain",
        type=int,
        choices=sorted(GAINS),
        help="default 4, or the calibration's",
    )
    p.add_argument(
        "--integration-ms",
        type=float,
        help="default 12 (multiples of 2.4), or the calibration's; keep it "
        "below the control loop period (20 ms in line_follow.py)",
    )
    args = p.parse_args()

    import pigpio

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit(
            "Can't connect to pigpiod. Start it with: sudo systemctl start pigpiod"
        )
    if args.calibrate:
        gain = args.gain or 4
        integration = args.integration_ms or 12
    else:
        gain, integration = args.gain, args.integration_ms
    sensor = LineSensor(
        pi,
        gain=gain,
        integration_ms=integration,
        cal_path=None if args.calibrate else CAL_PATH,
    )
    try:
        if args.calibrate and not calibrate(sensor):
            return
        live(sensor)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        sensor.close()
        pi.stop()


if __name__ == "__main__":
    main()
