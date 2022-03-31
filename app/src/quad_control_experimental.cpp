#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <math.h>
#include <queue>
#include <thread>
#include <time.h>

// Start streaming with
// mavlink stream -r 50 -s ESC_STATUS -d /dev/ttyACM0

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/shell/shell.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// yaml
#include <yaml-cpp/yaml.h>

// helpers
#include "mavsdk_helper.h"
#include "yaml_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/* CONSTANTS */
const float g = 9.81;                  // gravitational acceleration [m_s2]
const float quadcopter_mass = 1.5;     // Quadcopter mass [kg]
const float max_thrust = 4 * 8.9764;   // maximal thrust [N]
const float quad_rotor_radius = 0.12;  // Quadcopter rotor radius [m]
const float quad_rotor_distance = 0.3; // Quadcopter rotor distance [m]

/* GLOBAL VARIABLES */
std::vector<int> motorSpeeds{0, 0, 0, 0};
/* FUNCTION DECLARATIONS */

// void MotorSpeedCallback(const mavlink_message_t &ESC_STATUS) {
//   std::cout << "callback called" << std::endl;
//   for (int i = 0; i < sizeof(ESC_STATUS.payload64); i++) {
//     std::cout << "i:" << i << " " << ESC_STATUS.payload64[i] << std::endl;
//   }
// }

std::function<void(const mavlink_message_t &)> MotorSpeedCallback =
    [](const mavlink_message_t &raw_msg) {
      std::cout << "callback called" << std::endl;
      for (int i = 0; i < sizeof(raw_msg.payload64); i++) {
        std::cout << "i:" << i << " " << raw_msg.payload64[i] << std::endl;
      }
    };

std::function<void(std::string)> shellCallback = [](std::string msg) {
  std::cout << msg;
};
// thrust-throttle relation (linear)
float thrust_to_throttle(float thrust) {
  if (thrust > max_thrust) {
    return 1;
  }
  if (thrust < 0) {
    return 0;
  }
  return (0.02394 * thrust + 0.1644);
}

// reference generation
void trajectory_generator(float t, Eigen::Vector3f &pos,
                          Eigen::Vector3f &pos_ref, float &yaw_ref) {
  if (t > 0 && t <= 20) { // takeoff
    pos_ref(0) = 0;
    pos_ref(1) = 0;
    pos_ref(2) = 2.00;
    yaw_ref = 0.0;
  }
  // step response
  if (t > 25 && t <= 55) {
    pos_ref(0) = 0;
    pos_ref(1) = 0;
    pos_ref(2) = 0.05;
    yaw_ref = 0.0;
  }
  // if (t > 15 && t <= 45) { // fly circles
  //   pos_ref(0) = std::cos(params::circle_frequency * t * (2.0 * M_PI));
  //   pos_ref(1) = std::sin(params::circle_frequency * t * (2.0 * M_PI));
  //   pos_ref(2) = 2.0;
  //   yaw_ref = 0.0;
  // }
  if (t > 55) { // land
    std::cout << "landing now" << std::endl;
    pos_ref(0) = pos(0);
    pos_ref(1) = pos(1);
    pos_ref(2) = 0.0;
    yaw_ref = 0.0;
  }
}

// Cheeseman compensator
float CheesemanCompensator(float throttle_ref, float z) {
  return throttle_ref / (1.0 - std::pow((quad_rotor_radius / (4 * z)), 2));
}

// Nobahari compensator (R_eq = 2.5*R)
float NobahariCompensator(float throttle_ref, float z) {
  return throttle_ref /
         (1.0 - std::pow((2.5 * quad_rotor_radius / (4 * z)), 2));
}

// Hayden compensator
float HaydenCompensator(float throttle_ref, float z) {
  return throttle_ref * std::pow(0.9926 + 0.03794 * 4 * quad_rotor_radius *
                                              quad_rotor_radius / (z * z),
                                 2.0 / 3.0);
}

// Sanchez compensator
float SanchezCompensator(float throttle_ref, float z) {
  float d = quad_rotor_distance;
  float R = quad_rotor_radius;
  return throttle_ref /
         (1 - std::pow(R / (4 * z), 2) -
          R * R * (z / std::pow(std::pow(d * d + 4 * z * z, 3), 0.5)) -
          0.5 * R * R *
              (z / std::pow(std::pow(2 * d * d + 4 * z * z, 3), 0.5)));
}

int main(int argc, char **argv) {
  /* LOAD YAML PARAMETERS */
  set_parameters("app/parameters/params.yaml");

  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  std::string Name = "temp";
  myLog.open("log/" + Name + ".csv");
  std::cout << "Started logging to log/" << Name << ".csv\n";

  /* INITIALIZE MAVSDK */
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  auto action = Action{system};              // for arming / disarming etc
  auto offboard = Offboard{system};          // for offboard control
  auto telemetry = Telemetry{system};        // for telemetry services
  auto mavlink = MavlinkPassthrough{system}; // for direct mavlink messages
  auto shell = Shell{system};                // to talk with the systems shell

  // enable motor speed streaming
  std::string msg = "mavlink stream -r 50 -s ESC_STATUS ";

  shell.subscribe_receive(shellCallback);
  sleep_for(milliseconds(50));
  shell.send(msg);
  sleep_for(milliseconds(1000));
  shell.send("mavlink status streams");
  // subscribe to motor commands
  std::cout << "subscribing to message" << std::endl;
  mavlink.subscribe_message_async(290, MotorSpeedCallback);

  std::cout << "System is ready\n";

  /* ARM QUADCOPTER */
  const auto arm_result = action.arm();
  std::cerr << "Arming Result: " << arm_result << '\n';

  /* TAKEOFF (only needed for positon, velocity and acceleration control) */
  // const auto takeoff_result = action.takeoff();
  // std::cerr << "Takeoff Result: " << takeoff_result << '\n';
  // sleep_for(seconds(15));

  /* SEND OFFBOARD ONCE BEFORE STARTING (otherwise it will be rejected) */
  // velocity command
  // Offboard::VelocityNedYaw vel_cmd{};
  // offboard.set_velocity_ned(vel_cmd);

  // acceleration command
  // Offboard::AccelerationNed acc_cmd{};
  // offboard.set_acceleration_ned(acc_cmd);

  // attitude command
  Offboard::Attitude att_cmd{};
  offboard.set_attitude(att_cmd);

  /* STARTING OFFBOARD */
  Offboard::Result offboard_result = offboard.start();
  std::cerr << "Offboard Result: " << offboard_result << '\n';

  /* INITIALIZE VARIABLES */

  // reference values
  Eigen::Vector3f pos_ref(0, 0, 0);
  Eigen::Vector3f vel_ref(0, 0, 0);
  Eigen::Vector3f acc_ref(0, 0, 0);
  Eigen::Vector3f x_b_ref(0, 0, 0);
  Eigen::Vector3f y_b_ref(0, 0, 0);
  Eigen::Vector3f z_b_ref(0, 0, 0);
  Eigen::Matrix3f body_frame_ref;
  Eigen::Vector3f euler_ref(0, 0, 0);
  float yaw_ref = 0;

  // current states
  Eigen::Vector3f pos(0, 0, 0);                  // position
  Eigen::Vector3f vel(0, 0, 0);                  // velocity
  Eigen::Quaternion<float> att_quat(0, 0, 0, 0); // attitude quaternion
  Eigen::Vector3f att_euler(0, 0, 0);            // attitude euler angles
  Eigen::Matrix3f body_frame;                    // rotation matrix

  // controller errors
  Eigen::Vector3f pos_p_error(0, 0, 0);
  Eigen::Vector3f vel_p_error(0, 0, 0);
  Eigen::Vector3f vel_p_error_last(0, 0, 0);
  Eigen::Vector3f vel_i_error(0, 0, 0);
  Eigen::Vector3f vel_d_error(0, 0, 0);

  float t = 0;
  const float T_s_sec = float(params::T_s) / 1000.0;

  for (int i = 0;; i++) { // control loop at 50Hz
    // indices -> real-time
    t = float(i * params::T_s) / 1000.0;

    /* CURRENT STATE */
    // current position
    pos(0) = telemetry.position_velocity_ned().position.north_m;
    pos(1) = telemetry.position_velocity_ned().position.east_m;
    pos(2) = -telemetry.position_velocity_ned().position.down_m;
    // current velocity
    vel(0) = telemetry.position_velocity_ned().velocity.north_m_s;
    vel(1) = telemetry.position_velocity_ned().velocity.east_m_s;
    vel(2) = -telemetry.position_velocity_ned().velocity.down_m_s;
    // current orientation (euler angles)
    att_euler(0) = telemetry.attitude_euler().roll_deg;
    att_euler(1) = telemetry.attitude_euler().pitch_deg;
    att_euler(2) = telemetry.attitude_euler().yaw_deg;
    // current orientation (quaternion)
    att_quat.w() = telemetry.attitude_quaternion().w;
    att_quat.x() = telemetry.attitude_quaternion().x;
    att_quat.y() = telemetry.attitude_quaternion().y;
    att_quat.z() = telemetry.attitude_quaternion().z;
    //  body frame (rotation matrix)
    body_frame = att_quat.toRotationMatrix();

    // debug console outstream
    // std::cout << "x: " << pos(0) << "\ty: " << pos(1) << "\tz: " << pos(2)
    //           << "\tvx: " << vel(0) << "\tvy: " << vel(1) << "\tvz: " <<
    //           vel(2)
    //           << std::endl;

    /* TRAJECTORY GENERATION */
    trajectory_generator(t, pos, pos_ref, yaw_ref);

    /* POSITION CONTROLLER */
    // proportional position error
    pos_p_error = pos_ref - pos;
    // desired velocity
    vel_ref(0) = params::P_pos_XY * pos_p_error(0);
    vel_ref(1) = params::P_pos_XY * pos_p_error(1);
    vel_ref(2) = params::P_pos_Z * pos_p_error(2); // different gain for Z-error

    // check maximum velocities and constrain.
    if (vel_ref(0) > params::max_vel_XY) {
      vel_ref(0) = params::max_vel_XY;
    }
    if (vel_ref(0) < -params::max_vel_XY) {
      vel_ref(0) = -params::max_vel_XY;
    }
    if (vel_ref(1) > params::max_vel_XY) {
      vel_ref(1) = params::max_vel_XY;
    }
    if (vel_ref(1) < -params::max_vel_XY) {
      vel_ref(1) = -params::max_vel_XY;
    }
    if (vel_ref(2) > params::max_vel_Z_UP) {
      vel_ref(2) = params::max_vel_Z_UP;
    }
    if (vel_ref(2) < -params::max_vel_Z_DOWN) {
      vel_ref(2) = -params::max_vel_Z_DOWN;
    }

    /* VELOCITY CONTROLLER */
    // last proportional velocity error
    vel_p_error_last = vel_p_error;
    // proportional velocity error
    vel_p_error = vel_ref - vel;
    // integrative velocity error
    vel_i_error += vel_p_error * T_s_sec;
    // derivative velocity error
    vel_d_error = (vel_p_error - vel_p_error_last) / T_s_sec;
    // desired acceleration
    acc_ref(0) = params::P_vel_XY * vel_p_error(0) +
                 params::I_vel_XY * vel_i_error(0) +
                 params::D_vel_XY * vel_d_error(0);
    acc_ref(1) = params::P_vel_XY * vel_p_error(1) +
                 params::I_vel_XY * vel_i_error(1) +
                 params::D_vel_XY * vel_d_error(1);
    acc_ref(2) = params::P_vel_Z * vel_p_error(2) +
                 params::I_vel_Z * vel_i_error(2) +
                 params::D_vel_Z * vel_d_error(2); // different gain for Z-error

    /* CONVERSION TO ANGLES AND THRUST */
    // add gravitational acceleration
    acc_ref(2) = acc_ref(2) - g;

    // y-vector of global coordinte system turned around yaw_ref
    Eigen::Vector3f y_c(-std::sin(yaw_ref), std::cos(yaw_ref), 0);

    // find reference body frame. For more info see:
    // (https://github.com/uzh-rpg/rpg_quadrotor_control/blob/master/documents/theory_and_math/theory_and_math.pdf)
    z_b_ref = acc_ref;
    z_b_ref.normalize();
    x_b_ref = y_c.cross(z_b_ref);
    x_b_ref.normalize();
    y_b_ref = z_b_ref.cross(x_b_ref);

    // put reference body frame vectors into a matrix
    body_frame_ref.col(0) = x_b_ref;
    body_frame_ref.col(1) = y_b_ref;
    body_frame_ref.col(2) = z_b_ref;

    // calculate euler angles from rotation matrix
    euler_ref = body_frame_ref.eulerAngles(0, 1, 2);

    // project thurst onto body frame z-axis
    float acc_proj_z_b = acc_ref.dot(body_frame.col(2));
    float thrust_ref = (acc_proj_z_b)*quadcopter_mass; // F=M*a
    // thrust_ref = CheesemanCompensator(thrust_ref, pos(2)); // GE compensator
    float throttle_ref = thrust_to_throttle(thrust_ref);

    /* COMMANDS TO PX4 */
    // velocity commands (negative sign to account for xyz -> NED coordinate
    // change)
    //  vel_cmd.north_m_s = v_ref(0);
    //  vel_cmd.east_m_s = v_ref(1);
    //  vel_cmd.down_m_s = -v_ref(2);
    //  offboard.set_velocity_ned(vel_cmd);

    // acceleration commands (negative sign to account for xyz -> NED coordinate
    // change)
    // acc_cmd.north_m_s2 = acc_ref(0);
    // acc_cmd.east_m_s2 = acc_ref(1);
    // acc_cmd.down_m_s2 = -acc_ref(2);
    // offboard.set_acceleration_ned(acc_cmd);

    // attitude commands (negative sign to account for xyz -> NED coordinate
    // change)
    att_cmd.roll_deg = -euler_ref(0) * (180.0 / M_PI);
    att_cmd.pitch_deg = -euler_ref(1) * (180.0 / M_PI);
    att_cmd.yaw_deg = -euler_ref(2) * (180.0 / M_PI);
    att_cmd.thrust_value = throttle_ref;
    offboard.set_attitude(att_cmd);

    // debug console outstream
    // std::cout << "ro: " << -euler_ref(0) * (180.0 / M_PI)
    //           << "\tpi: " << -euler_ref(1) * (180.0 / M_PI)
    //           << "\tya: " << -euler_ref(2) * (180.0 / M_PI)
    //           << "\tth: " << throttle_ref << std::endl;

    /* LOGGING*/
    // t, p_ref, p, rpy
    // if (t > params::T_log) { // possibility to wait for transients to fade
    // away
    //   myLog << t << "," << pos_ref(0) << "," << pos_ref(1) << "," <<
    //   pos_ref(2)
    //         << "," << pos(0) << "," << pos(1) << "," << pos(2) << ","
    //         << att_euler(0) << "," << att_euler(1) << "," << att_euler(2)
    //         << "\n";
    // }

    // t, x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw, ctrls
    if (t > 20 && t <= 55) { //(t > params::T_log) { // possibility to wait for
                             // transients to fade away
      if (telemetry.actuator_control_target().controls.size() != 0) {
        myLog << t - 10.0 << ","
              << telemetry.position_velocity_ned().position.north_m << ","
              << telemetry.position_velocity_ned().position.east_m << ","
              << -telemetry.position_velocity_ned().position.down_m << ","
              << telemetry.position_velocity_ned().velocity.north_m_s << ","
              << telemetry.position_velocity_ned().velocity.east_m_s << ","
              << -telemetry.position_velocity_ned().velocity.down_m_s << ","
              << telemetry.attitude_euler().roll_deg << ","
              << telemetry.attitude_euler().pitch_deg << ","
              << telemetry.attitude_euler().yaw_deg << ","
              << telemetry.attitude_angular_velocity_body().roll_rad_s << ","
              << telemetry.attitude_angular_velocity_body().pitch_rad_s << ","
              << telemetry.attitude_angular_velocity_body().yaw_rad_s << ","
              << telemetry.actuator_control_target().controls.at(0) << ","
              << telemetry.actuator_control_target().controls.at(1) << ","
              << telemetry.actuator_control_target().controls.at(2) << ","
              << telemetry.actuator_control_target().controls.at(3) << "\n";
      }
    }

    /* SLEEP */
    sleep_for(milliseconds(params::T_s)); // 50Hz
  }

  /* DISARM QUADCOPTER */
  sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';

  return 0;
}
