import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import math
import os
import csv
import statistics


colors = ["red", "blue", "orange", "green",
          "orange", "grey", "olive", "cyan", "brown", "purple"]

ref_color = "black"  # reference color
curve_color = "red"  # measurement color
# Plot initialization
fig_xyz = plt.figure()  # xyz plot
z_plt = fig_xyz.add_subplot(111)

# fig_thrust = plt.figure()  # xyz plot
# thrust_plt = fig_thrust.add_subplot(111)
# Data initialization / Read Data

# df = pd.read_csv(sys.argv[2])

line = [0]*len(sys.argv)
header_list = [
    "t",
    "x_ref", "y_ref", "z_ref",
    "x", "y", "z",
    # "vx", "vy", "vz",
    "roll", "pitch", "yaw"
    # ,"c_r", "c_p", "c_y", "c_t"
]

for i in range(1, len(sys.argv)):
    df = pd.read_csv(sys.argv[i])
    time = pd.read_csv(sys.argv[i], usecols=["t"],
                       names=header_list).to_numpy()
    p_ref = pd.read_csv(sys.argv[i], usecols=[
                        "x_ref", "y_ref", "z_ref"], names=header_list).to_numpy()
    p = pd.read_csv(sys.argv[i], usecols=["x", "y", "z"],
                    names=header_list).to_numpy()
    # v = pd.read_csv(sys.argv[i], usecols=["vx", "vy",
    #                                      "vz"], names=header_list).to_numpy()
    # rpy = pd.read_csv(sys.argv[i], usecols=[
    #     "roll", "pitch", "yaw"], names=header_list).to_numpy()
   # ctrl = pd.read_csv(sys.argv[i], usecols=[
    #                   "c_r", "c_p", "c_y", "c_t"], names=header_list).to_numpy()
    z_plt.scatter(p_ref[:, 0], p_ref[:, 1], marker="o", s=1, c=ref_color)
    z_plt.scatter(p[:, 0], p[:, 1], marker="o", s=1, c=curve_color)

    # t-z plot
    # z_plt.scatter(time, p_ref[:, 2], marker="o", s=1, c=ref_color)
    # if(i == 1):
    #     low_line = z_plt.scatter(time/1000, p[:, 2], marker="o", s=1, c="blue")
    #     low_line.set_label('Quadcopter in ground effect')
    #     reference = z_plt.scatter(
    #         time/1000, p_ref[:, 2], marker="o", s=1, c=ref_color)
    #     reference.set_label('Reference')
    # if(i == 2):
    #     high_line = z_plt.scatter(
    #         time/1000, p[:, 2]-1.0, marker="o", s=1, c=curve_color)
    #     high_line.set_label('Quadcopter out of ground effect (-1 m)')

    # # t-thrust plot
    # thrust_plt.scatter(time, rpy[:, 1], marker="o", s=1, c=curve_color)


z_plt.set_xlabel("X [m]")
z_plt.set_ylabel("Y [m]")
# z_plt.legend()
plt.savefig("plot.svg")

# thrust_plt.set_xlabel("time [s]")
# thrust_plt.set_ylabel("thrust [mm]")

# Display Plots
plt.show()
