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

#df = pd.read_csv(sys.argv[2])


header_list = [
    "t",
    "z_ref",
    "z",
]

df = pd.read_csv(sys.argv[1])
Curve1 = pd.read_csv(
    sys.argv[1], names=header_list
).to_numpy()
z_plt.scatter(Curve1[:, 0], Curve1[:, 2], marker="o", s=1, c=ref_color)
z_plt.scatter(Curve1[:, 0], Curve1[:, 1], marker="o", s=1, c=curve_color)

df = pd.read_csv(sys.argv[2])
Curve2 = pd.read_csv(
    sys.argv[2], names=header_list
).to_numpy()
#z_plt.scatter(Curve2[:, 0], Curve2[:, 2], marker="o", s=1, c=ref_color)
z_plt.scatter(Curve2[:, 0], Curve2[:, 1]-0.9, marker="o", s=1, c=curve_color)


z_plt.set_xlabel("time [s]")
z_plt.set_ylabel("Z [mm]")


# Display Plots
plt.show()
