import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from scipy.optimize import curve_fit

# paths
path = "../log/traj_log.csv"  # data of hovering on multiple altitudes for T_IGE
# data of simple hovering on one altitude for T_OGE
hover_path = "../log/hover_log.csv"

# initialize plot
fig = plt.figure(figsize=(20, 8))  # xyz plot
ax = fig.add_subplot(111)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

# rpm-thrust relation (either from datasheet or experimental)


def rpmToThrust(rpm):
    return 1.1382941*(10**-7)*np.square(rpm)


# read in data
headerList = ["t", "x", "y", "z", "vx", "vy", "vz", "ro", "pi", "ya", "vro", "vpi", "vya", "rpm1",
              "rpm2", "rpm3", "rpm4", "volt1", "volt2", "volt3", "volt4", "amp1", "amp2", "amp3", "amp4"]
time = pd.read_csv(path, usecols=[
    "t"], names=headerList).to_numpy()
pos = pd.read_csv(path, usecols=[
                  "x", "y", "z"], names=headerList).to_numpy()
rpm = pd.read_csv(
    path, usecols=["rpm1", "rpm2", "rpm3", "rpm4"], names=headerList).to_numpy()

# determine signal length
signal_length = np.shape(time)[0]

# find out of ground effect (OGE) thrust
rpm_cal = pd.read_csv(
    hover_path, usecols=["rpm1", "rpm2", "rpm3", "rpm4"], names=headerList).to_numpy()
thrust_cal = np.sum(rpmToThrust(rpm_cal), axis=1)  # - mass*g
T_oge = np.average(thrust_cal)

# remove nan's
pos = np.nan_to_num(pos, copy=True, nan=0.0)  # [m]
rpm = np.nan_to_num(rpm, copy=True, nan=0.0)  # [1/min]

# filter data (if result is unsatisfying, the savgol-windows might need adjustment)
for i in range(2):
    pos[:, i] = savgol_filter(pos[:, i], 50, 3)
for i in range(3):
    rpm[:, i] = savgol_filter(rpm[:, i], 500, 3)

# rpm to thrust (according to relation)
T_motor_real = rpmToThrust(rpm)

# sum up the thrust over all motors
T_ige = np.sum(T_motor_real, axis=1)


cut_front = 2000
cut_back = 1000

# plot data
ax.plot(pos[cut_front:signal_length-cut_back, 2], T_oge/T_ige[cut_front:signal_length-cut_back],
        color="blue", label="flight data")

# perform data regression
x_data = pos[cut_front:signal_length-cut_back, 2]
y_data = T_oge/T_ige[cut_front:signal_length-cut_back]

# ansatz function


def f_ansatz(x, a, b, c):
    return a * np.exp(- b * x) + c


popt, pcov = curve_fit(f_ansatz, x_data, y_data)
print(popt)

# plot fit
plt.plot(x_data, f_ansatz(x_data, *popt), 'r-',
         label='fit: %5.3f*exp(-%5.3f*z)+ %5.3f' % tuple(popt))

ax.set_xlabel("z [m]", fontsize=20)
ax.set_ylabel("$T_{IGE}$ / $T_{OGE}$ [s]", fontsize=20)
plt.legend(prop={'size': 20})
plt.savefig("static_compensator.svg")
plt.savefig("static_compensator.pdf")
plt.show()
