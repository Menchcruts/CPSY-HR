#!/usr/bin/env python3
"""
Test script for TCS34725 RGB color sensor + SSD1306 OLED display
on a Raspberry Pi Zero.

Wiring (both devices share the same I2C bus, different addresses,
so SDA/SCL are wired in parallel to both boards):

  TCS34725      Pi Zero
  VIN     ->    3.3V
  GND     ->    GND
  SDA     ->    GPIO2 (SDA / physical pin 3)
  SCL     ->    GPIO3 (SCL / physical pin 5)

  SSD1306       Pi Zero
  VCC     ->    3.3V
  GND     ->    GND
  SDA     ->    GPIO2 (SDA / physical pin 3)
  SCL     ->    GPIO3 (SCL / physical pin 5)

Before running, enable I2C (raspi-config -> Interface Options -> I2C)
and check both devices show up:
  sudo i2cdetect -y 1
  (expect 0x29 for the TCS34725, 0x3C for the SSD1306)

Install dependencies:
  sudo pip3 install --break-system-packages \
      adafruit-circuitpython-tcs34725 \
      adafruit-circuitpython-ssd1306 \
      pillow
"""

import time

import board
import busio
import adafruit_tcs34725
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

# ---- Setup -----------------------------------------------------------

i2c = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 100  # ms; lower = faster updates, noisier readings
sensor.gain = 4                # 1, 4, 16, or 60 - raise this in dim light

WIDTH, HEIGHT = 128, 64
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)

font = ImageFont.load_default()

# ---- Main loop ---------------------------------------------------------


def main():
    print("Reading TCS34725, showing on OLED - Ctrl+C to stop\n")
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

            image = Image.new("1", (WIDTH, HEIGHT))
            draw = ImageDraw.Draw(image)

            draw.text((0, 0), "TCS34725 Reading", font=font, fill=255)
            draw.text((0, 16), f"R: {r}", font=font, fill=255)
            draw.text((0, 26), f"G: {g}", font=font, fill=255)
            draw.text((0, 36), f"B: {b}", font=font, fill=255)
            draw.text((0, 46), f"Lux: {lux:.1f}", font=font, fill=255)
            draw.text((0, 56), f"CT: {color_temp:.0f}K", font=font, fill=255)

            oled.image(image)
            oled.show()

            time.sleep(0.5)
    except KeyboardInterrupt:
        oled.fill(0)
        oled.show()
        print("\nStopped, display cleared.")


if __name__ == "__main__":
    main()
