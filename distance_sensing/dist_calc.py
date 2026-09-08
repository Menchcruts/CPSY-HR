import numpy as np
from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)


data = []
for _ in range(10):
    real_distance = float(input("Real distance (cm): "))
    nr_measurements = 1000
    for i in range(nr_measurements):
        print(f"[{i+1}/{nr_measurements} ({real_distance}cm)] - Measuring...")
        v = sensor.value * 3.3
        data.append((v, real_distance))

print("Finished measuring")
print("Calculating constants...")

V, D = zip(*data)

logV = np.log(V)
logD = np.log(D)

n, logA = np.polyfit(logV, logD, 1)
A = np.exp(logA)

print(f"D = {A:.3f} * V^{n:.3f}")
print(f"A = {A:.3f}")
print(f"n = {n:.3f}")
