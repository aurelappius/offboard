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

fig_thrust = plt.figure()  # xyz plot
thrust_plt = fig_thrust.add_subplot(111)
# Data initialization / Read Data

#df = pd.read_csv(sys.argv[2])


header_list = [
    "t",
    "x_ref", "y_ref", "z_ref",
    "x", "y", "z",
    "vx", "vy", "vz",
    "roll", "pitch", "yaw",
    "c_r", "c_p", "c_y", "c_t"
]

for i in range(1, len(sys.argv)):
    df = pd.read_csv(sys.argv[i])
    time = pd.read_csv(sys.argv[i], usecols=["t"],
                       names=header_list).to_numpy()
    p_ref = pd.read_csv(sys.argv[i], usecols=[
                        "x_ref", "y_ref", "z_ref"], names=header_list).to_numpy()
    p = pd.read_csv(sys.argv[i], usecols=["x", "y", "z"],
                    names=header_list).to_numpy()
    v = pd.read_csv(sys.argv[i], usecols=["vx", "vy",
                                          "vz"], names=header_list).to_numpy()
    rpy = pd.read_csv(sys.argv[i], usecols=[
                      "roll", "pitch", "yaw"], names=header_list).to_numpy()
    ctrl = pd.read_csv(sys.argv[i], usecols=[
                       "c_r", "c_p", "c_y", "c_t"], names=header_list).to_numpy()

    # t-z plot
    z_plt.scatter(time, p_ref[:, 2], marker="o", s=1, c=ref_color)
    z_plt.scatter(time, p[:, 2], marker="o", s=1, c=curve_color)
    # t-thrust plot
    thrust_plt.scatter(time, rpy[:, 1], marker="o", s=1, c=curve_color)


z_plt.set_xlabel("time [s]")
z_plt.set_ylabel("Z [mm]")

thrust_plt.set_xlabel("time [s]")
thrust_plt.set_ylabel("thrust [mm]")

# Display Plots
plt.show()
