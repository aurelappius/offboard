# Step responses

This data is from the quadcopter doing step responses from different heights:

HIGH_0*.csv: Step response from z=2.1m to z=1.1m
LOW_0*.csv: Step response from z=1.05m to z=0.05m
CHEESEMAN_0*.csv: Step response from z=1.05m to z=0.05m with a feedforward Cheeseman compensator. The Cheeseman compensator adjust the thrust according to the following equation: T_adj / T_0 = 1/(1-(R/4z)^2) where R is the rotor radius and z the height.

The Columns in the csv files are the following:
 * time
 * x
 * y
 * z
 * v_x
 * v_y
 * v_z
 * roll
 * pitch
 * yaw
 * v_roll
 * v_pitch
 * v_yaw
 * roll_command
 * pitch_command
 * yaw_command
 * thrust_command
