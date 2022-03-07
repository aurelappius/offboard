import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import math
import os
import csv
import statistics

# Parameters
Start = 0  # start frame
N = 10000  # how many frames

ref_color = "black"  # reference color
curve_color = "red"  # measurement color
# Plot initialization
fig_xyz = plt.figure()  # xyz plot
z_plt = fig_xyz.add_subplot(111)

# Data initialization / Read Data
pathToData = sys.argv[1]
df = pd.read_csv(pathToData)

header_list = [
    "t",
    "z_ref",
    "z",
]  # old format

t = pd.read_csv(
    pathToData, usecols=["t"], skiprows=Start, nrows=N, names=header_list
).to_numpy()

z_ref = pd.read_csv(
    pathToData, usecols=["z_ref"], skiprows=Start, nrows=N, names=header_list
).to_numpy()

z = pd.read_csv(
    pathToData, usecols=["z"], skiprows=Start, nrows=N, names=header_list
).to_numpy()


z_plt.scatter(t, z_ref, marker="o", s=1, c=ref_color)
z_plt.scatter(t, z, marker="o", s=1, c=curve_color)
z_plt.set_xlabel("time [s]")
z_plt.set_ylabel("Z [mm]")


# Display Plots
plt.show()
