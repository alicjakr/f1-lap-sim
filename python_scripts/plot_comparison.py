import pandas as pd
import matplotlib.pyplot as plt

sim = pd.read_csv("data/simulated_lap.csv")
ref = pd.read_csv("data/reference_lap.csv")

plt.plot(ref["Distance"], ref["Speed"], label="Reference (HAM 2018)")
plt.plot(sim["Distance"], sim["Speed"], label="Simulated")
plt.xlabel("Distance [m]")
plt.ylabel("Speed [km/h]")
plt.legend()
plt.savefig("data/comparison.png", dpi=150)