from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)

def distance(v: float) -> float:
    # return 35.785 * (v ** -1.181)
    # return 94.993 * (v ** -1.562)
    return 1638.806 * (v ** -5.054)

while True:
    v = sensor.value * 3.3
    print(f"raw: {sensor.value:.3f} - voltage: {v:.2f} V")
    dist = distance(v)
    print(f"Distance from object: {dist:.3f}cm")
    print()
    sleep(0.3)
