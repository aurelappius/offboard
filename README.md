# quad_control
This code was used in the Bachelor's thesis "Modeling and Control of Quadcopters in Ground Effect using
Physics Informed Machine Learning" by Aurel X. Appius.

The code contains two applications that are described in the following: 

## app_quad_control
This app can be used to control a quadcopter with ground effect compensation. The code should run on a companion computer that is connected to an autopilot that supports MAVLink communication. For more details, take a look at the thesis.

The code allows the deployment of any compensator in the compensator.h / compensator.cpp file.

Custom trajectories can also be added in the trajectory.h / trajecotry.cpp file.

## app_thrust_stand

The app "app_thrust_stand" can be used to find the throttle-thrust-rpm relation for the motors.

For a successfull experiment, one of the motors needs to be mounted on a test bench that measures the thrust. For more details, take a look at the thesis.

