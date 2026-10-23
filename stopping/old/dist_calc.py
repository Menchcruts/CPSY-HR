import numpy as np
from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)


data = []
nr_distances = 10
for d in range(nr_distances):
    real_distance = float(input(f"[{d+1}/{nr_distances}] - Real distance (cm): "))
    nr_measurements = 250
    for i in range(nr_measurements):
        print(f"[{i+1}/{nr_measurements} ({real_distance}cm)] - Measuring...\r", end="")
        v = sensor.value * 3.3
        data.append((v, real_distance))
    print()

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
