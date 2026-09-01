import time
import math
import board
import busio
import adafruit_icm20x
from gpiozero import LED

ic2 = busio.I2C(board.SCL, board.SDA)
icm = adafruit_icm20x.ICM20948(ic2)

north_led = LED(17)
level_led = LED(27)


while True:
    ax, ay, az = icm.acceleration
    pitch = math.degrees(
        math.atan2(-ax, math.sqrt(ay**2 + az**2))
    )
    roll = math.degrees(
        math.atan2(ay, az)
    )

    heading = math.degrees(math.atan2(icm.magnetic[1], icm.magnetic[0]))
    if heading < 0:
        heading += 360

    print(f"Heading: {heading:.2f}")
    print(f"Pitch: {pitch:.2f}")
    print(f"Roll: {roll:.2f}")

    if 0 <= heading <= 10 or 350 <= heading <= 360:
        print("North!")
        north_led.on()
    else:
        north_led.off()

    if (-10 <= pitch <= 10 and -10 <= roll <= 10):
        print("Level!")
        level_led.on()
    else:
        level_led.off()


    print("")

    time.sleep(0.5)
