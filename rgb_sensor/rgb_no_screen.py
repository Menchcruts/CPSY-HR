#!/usr/bin/env python3
"""
Test script for TCS34725 RGB color sensor on a Raspberry Pi Zero.

Wiring:

  TCS34725      Pi Zero
  VIN     ->    3.3V
  GND     ->    GND
  SDA     ->    GPIO2 (SDA / physical pin 3)
  SCL     ->    GPIO3 (SCL / physical pin 5)

Before running, enable I2C (raspi-config -> Interface Options -> I2C)
and check the sensor shows up:
  sudo i2cdetect -y 1
  (expect 0x29 for the TCS34725)

Install dependencies:
  sudo pip3 install --break-system-packages adafruit-circuitpython-tcs34725
"""

import time

import board
import busio
import adafruit_tcs34725

# ---- Setup -----------------------------------------------------------

i2c = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 100  # ms; lower = faster updates, noisier readings
sensor.gain = 4                # 1, 4, 16, or 60 - raise this in dim light

# ---- Main loop ---------------------------------------------------------


def main():
    print("Reading TCS34725 - Ctrl+C to stop\n")
    try:
        while True:
            r, g, b = sensor.color_rgb_bytes
            try:
                lux = sensor.lux
                color_temp = sensor.color_temperature
            except Exception:
                # can briefly divide-by-zero in very low light
                lux, color_temp = 0.0, 0.0

            print(f"R={r:3d} G={g:3d} B={b:3d}  Lux={lux:6.1f}  CT={color_temp:5.0f}K")

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
