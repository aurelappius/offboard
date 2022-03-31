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
compensators = ["Cheeseman compensation", "Hayden compensation",
                "Sanchez compensation", "Nobahari compensation"]
colors = ["red", "blue", "green", "olive",
          "orange", "grey", "cyan", "brown", "purple"]
plot_flag = [True, True, True, True, True]

R = 0.12
d = 0.
K_b = 2
z = np.linspace(0.15, 1, 2000)


Cheeseman = 1/(1-(R/(4*z))**2)
Hayden = (0.9926 + 0.03794*4*(R/z)**2)**(2/3)
Sanchez = 1/(1 - (R/(4*z))**2 - R**2 * z/(((d**2+4*z**2)**3)**0.5) - 0.5 * R **
             2 * z/(((2*d**2+4*z**2)**3)**0.5) - 2*R**2 * z/(((d**2+4*z**2)**3)**0.5)*K_b)
Nobahari = 1/(1-(2.5*R/(4*z))**2)
# / Cheeseman compensator
# float CheesemanCompensator(float throttle_ref, float z) {
#     return throttle_ref / (1.0 - std: : pow((quad_rotor_radius / (4 * z)), 2))
# }

# // Nobahari compensator(R_eq=2.5*R)
# float NobahariCompensator(float throttle_ref, float z) {
#     return throttle_ref /
#     (1.0 - std: : pow((2.5 * quad_rotor_radius / (4 * z)), 2))
# }

# // Hayden compensator
# float HaydenCompensator(float throttle_ref, float z) {
#     return throttle_ref * std:: pow(0.9926 + 0.03794 * 4 * quad_rotor_radius *
#                                      quad_rotor_radius / (z * z),
#                                      2.0 / 3.0)
# }

# // Sanchez compensator
# float SanchezCompensator(float throttle_ref, float z) {
#     float d = quad_rotor_distance
#     float R = quad_rotor_radius
#     return throttle_ref /
#     (1 - std:: pow(R / (4 * z), 2) -
#      R * R * (z / std:: pow(std: : pow(d * d + 4 * z * z, 3), 0.5)) -
#      0.5 * R * R *
#      (z / std: : pow(std: : pow(2 * d * d + 4 * z * z, 3), 0.5)))
# }

z_plt.plot(z, Cheeseman, c=colors[0], label=compensators[0])

z_plt.plot(z, Hayden, c=colors[1], label=compensators[1])

z_plt.plot(z, Sanchez, c=colors[2], label=compensators[2])

z_plt.plot(z, Nobahari, c=colors[3], label=compensators[3])


z_plt.set_xlabel("z [m]")
z_plt.set_ylabel("T_ige / T_oge [s]")
z_plt.legend(prop={'size': 30})
plt.savefig("plot.svg")
plt.show()
