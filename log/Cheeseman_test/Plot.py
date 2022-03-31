import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import math
import os
import csv
import statistics

# Plot initialization
fig_xyz = plt.figure()  # xyz plot
z_plt = fig_xyz.add_subplot(111)

# global variables
deviation = 1
n_compensator = int(len(sys.argv)/5)
compensators = ["Cheeseman compensation", "No compensation",
                "High Altitude (-1m)", "Sanchez compensation", "Hayden compensation"]
colors = ["red", "blue", "green", "olive",
          "orange", "grey", "cyan", "brown", "purple"]
plot_flag = [True, True, True, True, True]


# t, x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw, ctrls
header_list = [
    "t",
    "x", "y", "z",
    "vx", "vy", "vz",
    "roll", "pitch", "yaw",
    "vroll", "vpitch", "vyaw",
    "c_r", "c_p", "c_y", "c_t"
]

# initialize arrays
data = np.empty((len(sys.argv), 2000, 4))
data_avg = np.empty((5, 2000, 3))
data_std = np.empty((5, 2000, 3))

# read data
for i in range(0, len(sys.argv)-1):
    data_shape = np.shape(pd.read_csv(sys.argv[i+1]).to_numpy())
    data[i, 0:data_shape[0]+1, :] = pd.read_csv(sys.argv[i+1], usecols=["t", "x", "y", "z"],
                                                names=header_list).to_numpy()

# change height (-1.05 meters)
data[10:15, :, 1:4] = data[10:15, :, 1:4]-1.0

# calculate average
for i in range(0, n_compensator):
    data_avg[i, :, :] = np.mean(data[5*i:5*(i+1), :, 1:4], 0)
    data_std[i, :, :] = np.std(data[5*i:5*(i+1), :, 1:4], 0)


# plot curves
for i in range(0, n_compensator):
    if(plot_flag[i]):
        z_plt.plot(data[0, 0:1700, 0], data_avg[i,
                                                0:1700, 2], c=colors[i], label=compensators[i])
        z_plt.fill_between(data[0, 0:1700, 0], data_avg[i, 0:1700, 2] -
                           deviation*data_std[i, 0:1700, 2], data_avg[i, 0:1700, 2]+deviation*data_std[i, 0:1700, 2], color=colors[i], alpha=0.3)


z_plt.set_xlabel("x [m]")
z_plt.set_ylabel("t [s]")
z_plt.legend(prop={'size': 30})
plt.savefig("plot.svg")
plt.show()


# data_avg[i, :, :] = np.mean(data[0:5, :, 1:4], 0)
# data_avg[1, :, :] = np.mean(data[5:10, :, 1:4], 0)
# data_avg[2, :, :] = np.mean(data[10:15, :, 1:4], 0)
# data_avg[3, :, :] = np.mean(data[15:20, :, 1:4], 0)
# data_avg[4, :, :] = np.mean(data[20:25, :, 1:4], 0)


# calculate std deviation
# data_std[0, :, :] = np.std(data[0:5, :, 1:4], 0)
# data_std[1, :, :] = np.std(data[5:10, :, 1:4], 0)
# data_std[2, :, :] = np.std(data[10:15, :, 1:4], 0)
# data_std[3, :, :] = np.std(data[15:20, :, 1:4], 0)
# data_std[4, :, :] = np.std(data[20:25, :, 1:4], 0)
