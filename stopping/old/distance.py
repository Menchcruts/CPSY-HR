from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)

def distance(v: float, A: float, n: float) -> float:
    return A * (v ** n)

A = 56.351
n = -1.186

try:
    while True:
        v = sensor.value * 3.3
        print(f"raw: {sensor.value:.3f} - voltage: {v:.2f} V")
        dist = distance(v, A, n)
        print(f"Distance from object: {dist:.3f}cm")
        print()
        sleep(0.3)
except KeyboardInterrupt:
    print("Stopping")
    
