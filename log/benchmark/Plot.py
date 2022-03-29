from turtle import shape
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
# t, x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw, ctrls
header_list = [
    "t",
    "x", "y", "z",
    "vx", "vy", "vz",
    "roll", "pitch", "yaw",
    "vroll", "vpitch", "vyaw",
    "c_r", "c_p", "c_y", "c_t"
]


print(sys.argv[1])
print(sys.argv[6])
print(sys.argv[11])
data = np.empty((len(sys.argv), 2000, 4))

data_avg = np.empty((3, 2000, 3))

data_std = np.empty((3, 2000, 3))

for i in range(0, len(sys.argv)-1):
    data_shape = np.shape(pd.read_csv(sys.argv[i+1]).to_numpy())
    # print(data_shape)
    # print(np.shape(data[i, 0:data_shape[0]+1, 0]))
    data[i, 0:data_shape[0]+1, :] = pd.read_csv(sys.argv[i+1], usecols=["t", "x", "y", "z"],
                                                names=header_list).to_numpy()

# change height (-1.05 meters)
data[10:15, :, 1:4] = data[10:15, :, 1:4]-1.05

# calculate average
data_avg[0, :, :] = np.mean(data[0:5, :, 1:4], 0)
data_avg[1, :, :] = np.mean(data[5:10, :, 1:4], 0)
data_avg[2, :, :] = np.mean(data[10:15, :, 1:4], 0)

# calculate std deviation
data_std[0, :, :] = np.std(data[0:5, :, 1:4], 0)
data_std[1, :, :] = np.std(data[5:10, :, 1:4], 0)
data_std[2, :, :] = np.std(data[10:15, :, 1:4], 0)

deviation = 1

cheeseman = z_plt.plot(data[0, 0:1700, 0], data_avg[0,
                       0:1700, 2], c="red", label="Cheeseman compensation")
z_plt.fill_between(data[0, 0:1700, 0], data_avg[0, 0:1700, 2] -
                   deviation*data_std[0, 0:1700, 2], data_avg[0, 0:1700, 2]+deviation*data_std[0, 0:1700, 2], color="red", alpha=0.3)

# z_plt.plot(data[0, 0:1700, 0], data[0, 0:1700, 3], c="grey")
# z_plt.plot(data[1, 0:1700, 0], data[1, 0:1700, 3], c="grey")
# z_plt.plot(data[2, 0:1700, 0], data[2, 0:1700, 3], c="grey")
# z_plt.plot(data[3, 0:1700, 0], data[3, 0:1700, 3], c="grey")
# z_plt.plot(data[4, 0:1700, 0], data[4, 0:1700, 3], c="grey")
# z_plt.plot(data[5, 0:1700, 0], data[5, 0:1700, 3], c="grey")
# z_plt.plot(data[6, 0:1700, 0], data[6, 0:1700, 3], c="grey")
# z_plt.plot(data[7, 0:1700, 0], data[7, 0:1700, 3], c="grey")
# z_plt.plot(data[8, 0:1700, 0], data[8, 0:1700, 3], c="grey")
# z_plt.plot(data[9, 0:1700, 0], data[9, 0:1700, 3], c="grey")
# z_plt.plot(data[10, 0:1700, 0], data[10, 0:1700, 3], c="grey")
# z_plt.plot(data[11, 0:1700, 0], data[11, 0:1700, 3], c="grey")
# z_plt.plot(data[12, 0:1700, 0], data[12, 0:1700, 3], c="grey")
# z_plt.plot(data[13, 0:1700, 0], data[13, 0:1700, 3], c="grey")
# z_plt.plot(data[14, 0:1700, 0], data[14, 0:1700, 3], c="grey")


low = z_plt.plot(data[0, 0:1700, 0], data_avg[1, 0:1700,
                 2], c="blue", label="No compensation")
z_plt.fill_between(data[0, 0:1700, 0], data_avg[1, 0:1700, 2] -
                   deviation*data_std[1, 0:1700, 2], data_avg[1, 0:1700, 2]+deviation*data_std[1, 0:1700, 2], color="blue", alpha=0.3)
#low.set_label('no compensation')

high = z_plt.plot(data[0, 0:1700, 0], data_avg[2, 0:1700,
                  2], c="green", label="High Altitude")
z_plt.fill_between(data[0, 0:1700, 0], data_avg[2, 0:1700, 2] -
                   deviation*data_std[2, 0:1700, 2], data_avg[2, 0:1700, 2]+deviation*data_std[2, 0:1700, 2], color="green", alpha=0.3)
#high.set_label('high altitude')
# legend

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
z_plt.legend(prop={'size': 30})
plt.savefig("plot.svg")

# thrust_plt.set_xlabel("time [s]")
# thrust_plt.set_ylabel("thrust [mm]")

# Display Plots
plt.show()
