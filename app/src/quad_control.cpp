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

// FastDDS
#include "domain_participant.h"
#include "sub_callback.h"
#include "subscriber.h"
// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// parameters
#include "parameter_list.h"
// yaml
#include <yaml-cpp/yaml.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// constants
const float g = 9.81;                // m_s2
const float quadcopter_mass = 1.5;   // kg
const float max_thrust = 4 * 8.9764; // N

// thrust-throttle relation
float thrust_to_throttle(float thrust) {
  if (thrust > max_thrust) {
    return 1;
  }
  if (thrust < 0) {
    return 0;
  }
  return (0.02394 * thrust + 0.1644);
}

// Checks whether given yaml file exists
inline void yaml_file_check(std::string yaml_file) {

  try {
    if (std::filesystem::exists(yaml_file) == false)
      throw(yaml_file);
  } catch (std::string yaml_file) {
    std::cerr << "YAML file error: " << yaml_file << " does not exist" << '\n';
    std::exit(EXIT_FAILURE);
  }
}

inline void set_parameters(const std::string setpoint_path) {

  // check if yaml file exists
  yaml_file_check(setpoint_path);
  // Load yaml file containing gains
  YAML::Node commands_yaml = YAML::LoadFile(setpoint_path);

  // Set parameters
  params::SIM = commands_yaml["SIM"].as<bool>();
  params::v = commands_yaml["v"].as<float>();
  params::h_0 = commands_yaml["h_0"].as<float>();
  params::freq = commands_yaml["freq"].as<float>();
  params::amp = commands_yaml["amp"].as<float>();
  params::P_vel_XY = commands_yaml["P_vel_XY"].as<float>();
  params::I_vel_XY = commands_yaml["I_vel_XY"].as<float>();
  params::D_vel_XY = commands_yaml["D_vel_XY"].as<float>();
  params::P_pos_XY = commands_yaml["P_pos_XY"].as<float>();
  params::P_vel_Z = commands_yaml["P_vel_Z"].as<float>();
  params::I_vel_Z = commands_yaml["I_vel_Z"].as<float>();
  params::D_vel_Z = commands_yaml["D_vel_Z"].as<float>();
  params::P_pos_Z = commands_yaml["P_pos_Z"].as<float>();
  params::T_s = commands_yaml["T_s"].as<int>();
  params::k_h = commands_yaml["k_h"].as<double>();
  params::hover = commands_yaml["hover"].as<float>();
  params::T = commands_yaml["T"].as<int>();
}

void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk) {
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    }
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}

int main(int argc, char **argv) {

  // load yaml parameters
  set_parameters("app/parameters/params.yaml");

  // // FASTDDS
  // // Create participant. Arguments-> Domain id, QOS name
  // DefaultParticipant dp(0, "mocap_subscriber");

  // // Create publisher with msg type
  // DDSSubscriber mocap_sub(idl_msg::MocapPubSubType(), &sub::mocap_msg,
  //                         "mocap_srl_quad", dp.participant());
  // if (!params::SIM) {
  //   // activate mocap
  //   for (int i = 0; i < 10; i++) {
  //     mocap_sub.listener->wait_for_data();
  //     std::cout << "x= " << sub::mocap_msg.pose.position.x << std::endl;
  //   }
  //   std::cout << "initialized mocap" << std::endl;
  // }

  // LOGGING

  // std::ofstream myLog;

  // std::string Name_0 = "flight";
  // std::string suffix = "_nr_0";
  // std::string Name_h0 = "_h0_";
  // std::string val_h0 = std::to_string(params::h_0);
  // val_h0.erase(val_h0.find_last_not_of('0') + 2, std::string::npos);
  // std::string Name_v = "_v_";
  // std::string val_v = std::to_string(params::v);
  // val_v.erase(val_v.find_last_not_of('0') + 2, std::string::npos);
  // std::string Name_freq = "_freq_";
  // std::string val_freq = std::to_string(params::freq);
  // val_freq.erase(val_freq.find_last_not_of('0') + 2, std::string::npos);
  // std::string Name_amp = "_amp_";
  // std::string val_amp = std::to_string(params::amp);
  // val_amp.erase(val_amp.find_last_not_of('0') + 2, std::string::npos);
  // std::string Name_sim;
  // if (params::SIM == true) {
  //   Name_sim = "_SIM";
  // } else {
  //   Name_sim = "_REAL";
  // }
  // std::string Name = Name_0 + Name_h0 + val_h0 + Name_v + val_v + Name_freq +
  //                    val_freq + Name_amp + val_amp + Name_sim + suffix;

  // // check if file already exists
  // for (int i = 0;; i++) {
  //   Name.pop_back();
  //   Name = Name + std::to_string(i);
  //   std::filesystem::path f{"log/" + Name + ".csv"};
  //   if (!(std::filesystem::exists(f))) {
  //     break;
  //   }
  // }

  // myLog.open("log/" + Name + ".csv");

  // std::cout << "Started logging to log/" << Name << ".csv\n";

  // contdown [for testing]
  // for (int i = 3; i > 0; i--) {
  //   std::cout << "T minus " << i << std::endl;
  //   sleep_for(seconds(1));
  // }
  // std::cout << "LIFTOFF!" << std::endl;
  std::ofstream myLog;
  std::string Name = "temp";
  myLog.open("log/" + Name + ".csv");
  std::cout << "Started logging to log/" << Name << ".csv\n";

  /// MAVSDK
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

  // Instantiate plugins.
  auto action = Action{system};
  auto offboard = Offboard{system};
  auto telemetry = Telemetry{system};

  std::cout << "System is ready\n";

  const auto arm_result = action.arm();
  std::cerr << "Arming Result: " << arm_result << '\n';

  /* TAKEOFF (only needed for positon, velocity and acceleration control) */
  // const auto takeoff_result = action.takeoff();
  // std::cerr << "Takeoff Result: " << takeoff_result << '\n';
  // sleep_for(seconds(10));

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
  Eigen::Vector3f pos(0, 0, 0);             // position
  Eigen::Vector3f vel(0, 0, 0);             // velocity
  Eigen::Quaternion<float> att(0, 0, 0, 0); // attitude quaternion
  Eigen::Matrix3f body_frame;               // rotation matrix

  // controller errors
  Eigen::Vector3f pos_p_error(0, 0, 0);
  Eigen::Vector3f vel_p_error(0, 0, 0);
  Eigen::Vector3f vel_p_error_last(0, 0, 0);
  Eigen::Vector3f vel_i_error(0, 0, 0);
  Eigen::Vector3f vel_d_error(0, 0, 0);

  float t = 0;
  const float T_s_sec = float(params::T_s) / 1000.0;

  for (int i = 0;; i++) {
    // indices -> real-time
    t = float(i * params::T_s) / 1000.0;

    // trajectory generation
    trajectory_generator(pos_ref, yaw_ref);

    /* CURRENT STATE */
    // current position
    pos(0) = telemetry.position_velocity_ned().position.north_m;
    pos(1) = telemetry.position_velocity_ned().position.east_m;
    pos(2) = -telemetry.position_velocity_ned().position.down_m;
    // current velocity
    vel(0) = telemetry.position_velocity_ned().velocity.north_m_s;
    vel(1) = telemetry.position_velocity_ned().velocity.east_m_s;
    vel(2) = -telemetry.position_velocity_ned().velocity.down_m_s;
    // current orientation (quaternion)
    att.w() = telemetry.attitude_quaternion().w;
    att.x() = telemetry.attitude_quaternion().x;
    att.y() = telemetry.attitude_quaternion().y;
    att.z() = telemetry.attitude_quaternion().z;
    //  body frame (rotation matrix)
    body_frame = att.toRotationMatrix();

    /* POSITION CONTROLLER */
    // proportional position error
    pos_p_error = pos_ref - pos;
    // desired velocity
    vel_ref(0) = params::P_pos_XY * pos_p_error(0);
    vel_ref(1) = params::P_pos_XY * pos_p_error(1);
    vel_ref(2) = params::P_pos_Z * pos_p_error(2); // different gain for Z-error

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

    /* LOGGING*/
    // t, p_ref, p
    if (t > 0) { // wait for transients to fade away
      myLog << t << "," << pos_ref(0) << "," << pos_ref(1) << "," << pos_ref(2)
            << "," << pos(0) << "," << pos(1) << "," << pos(2) << "\n";
    }

    /* SLEEP */
    sleep_for(milliseconds(params::T_s)); // 50Hz
  }

  sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';

  return 0;
}
