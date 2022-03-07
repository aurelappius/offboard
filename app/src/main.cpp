#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
#include <time.h>
// include
#include "parameter_list.h"

// FastDDS
#include "domain_participant.h"
#include "sub_callback.h"
#include "subscriber.h"
// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// yaml
#include <yaml-cpp/yaml.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// constants
const double g = 9.81;

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
  params::P_vel = commands_yaml["P_vel"].as<float>();
  params::I_vel = commands_yaml["I_vel"].as<float>();
  params::D_vel = commands_yaml["D_vel"].as<float>();
  params::P_pos = commands_yaml["P_pos"].as<float>();
  params::T_s = commands_yaml["sample_time"].as<float>();
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

//
// Does Offboard control using NED co-ordinates.
//
// returns true if everything went well in Offboard control
//
bool offb_ctrl_ned(
    mavsdk::Offboard &offboard,
    DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> &mocap) {
  std::cout << "Starting Offboard velocity control in NED coordinates\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  const Offboard::VelocityNedYaw stay{};
  offboard.set_velocity_ned(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }

  // float x_ref = 0.0;
  // float y_ref = 1.0;
  // float z_ref = 1.5;
  // Offboard::PositionNedYaw msg;
  // msg.down_m=0;
  // msg.east_m=0;
  // msg.north_m=0;
  // while(true){
  //     mocap.listener->wait_for_data();
  //     msg.east_m = x_ref - sub::mocap_msg.pose.position.x;
  //     msg.north_m = y_ref - sub::mocap_msg.pose.position.y;
  //     msg.down_m = -( z_ref - sub::mocap_msg.pose.position.z );

  //     offboard.set_position_ned(msg);

  // }

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

// fastDDS test function
void test(DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> &mocap) {
  for (int i = 0; i < 10; i++) {
    mocap.listener->wait_for_data();
    std::cout << "x= " << sub::mocap_msg.pose.position.x << std::endl;
  }
}

bool offb_ctrl_acc(mavsdk::Offboard &offboard) {
  std::cout << "Starting Offboard acceleration control\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::AccelerationNed stay{};
  stay.east_m_s2 = 0;
  stay.north_m_s2 = 0;
  stay.down_m_s2 = 0;
  offboard.set_acceleration_ned(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard started\n";

  // takeoff
  std::cout << "Takeoff\n";
  Offboard::AccelerationNed takeoff{};
  takeoff.east_m_s2 = 0;
  takeoff.north_m_s2 = 0;
  takeoff.down_m_s2 = 0;
  offboard.set_acceleration_ned(takeoff);
  sleep_for(milliseconds(6000));

  // stay
  std::cout << "Hover\n";
  offboard.set_acceleration_ned(stay);
  sleep_for(milliseconds(6000));

  // land
  std::cout << "land\n";
  Offboard::AccelerationNed land{};
  land.east_m_s2 = 0;
  land.north_m_s2 = 0;
  land.down_m_s2 = 5;
  offboard.set_acceleration_ned(land);
  sleep_for(milliseconds(6000));

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

bool offb_ctrl_attitude_RPG(
    mavsdk::Offboard &offboard, mavsdk::Telemetry &telemetry,
    DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> &mocap) {

  std::cout << "Starting Offboard attitude control\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::Attitude stay{};
  stay.roll_deg = 0.0f;
  stay.pitch_deg = 0.0f;
  stay.yaw_deg = 0.0f;
  stay.thrust_value = 0.3f;
  offboard.set_attitude(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard started\n";

  // Dummy loop
  // std::chrono::steady_clock::time_point end =
  //     std::chrono::steady_clock::now() + seconds(params::T);
  // std::cout << "thrust" << std::endl;
  // while (std::chrono::steady_clock::now() < end) {
  //   Offboard::Attitude cmd{};
  //   cmd.roll_deg = 0.0f;
  //   cmd.pitch_deg = 0.0f;
  //   cmd.yaw_deg = 0.0f;
  //   cmd.thrust_value = 0.6f;
  //   offboard.set_attitude(cmd);
  //   sleep_for(milliseconds(50));
  // }
  // params
  Eigen::DiagonalMatrix<double, 3> K_pos(1, 1, 1);
  Eigen::DiagonalMatrix<double, 3> K_vel(1, 1, 1);
  Eigen::DiagonalMatrix<double, 3> D(0, 0, 0); // rotor drag

  // reference
  Eigen::Vector3d x_ref(0, 0, 2); // position
  Eigen::Vector3d v_ref(0, 0, 0); // velocity
  Eigen::Vector3d a_ref(0, 0, 0); // acceleration
  Eigen::Vector3d R_ref(0, 0, 0); // orientation
  double yaw = 0;                 // yaw [radians]

  // yaw enforce helpers
  Eigen::Vector3d x_c(cos(yaw), sin(yaw), 2);
  Eigen::Vector3d y_c(-sin(yaw), cos(yaw), 2);

  // unit vectors of world
  Eigen::Vector3d x_w(1, 0, 0); // position
  Eigen::Vector3d y_w(0, 1, 0); // position
  Eigen::Vector3d z_w(0, 0, 1); // position

  // control loop
  std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now() + seconds(params::T);

  float c_max = 1.5 * 230951;
  while (std::chrono::steady_clock::now() < end) {
    // current state (pos/vel)
    Eigen::Vector3d x(telemetry.position_velocity_ned().position.north_m,
                      telemetry.position_velocity_ned().position.east_m,
                      -telemetry.position_velocity_ned().position.down_m);
    Eigen::Vector3d v(telemetry.position_velocity_ned().velocity.north_m_s,
                      telemetry.position_velocity_ned().velocity.east_m_s,
                      -telemetry.position_velocity_ned().velocity.down_m_s);

    // rotation of quadcopter
    Eigen::Vector3d eul(telemetry.attitude_euler().roll_deg * (M_PI / 180.0),
                        telemetry.attitude_euler().pitch_deg * (M_PI / 180.0),
                        telemetry.attitude_euler().yaw_deg * (M_PI / 180.0));
    Eigen::Quaterniond quat(
        telemetry.attitude_quaternion().w, telemetry.attitude_quaternion().z,
        telemetry.attitude_quaternion().y, telemetry.attitude_quaternion().z);
    Eigen::Matrix3d ROT = quat.toRotationMatrix();

    // Body frame
    Eigen::Vector3d x_b = ROT.col(0); // position
    Eigen::Vector3d y_b = ROT.col(1); // position
    Eigen::Vector3d z_b = ROT.col(2); // position

    // desired acceleration
    Eigen::Vector3d a_fb = (-1) * K_pos * (x - x_ref) - K_vel * (v - v_ref);
    // Eigen::Vector3d a_dr = (-1) * (R_ref * D * R_ref.transpose()) * v_ref;
    Eigen::Vector3d a_dr(0, 0, 0);
    Eigen::Vector3d a_des = a_fb + a_des - a_dr + g * z_w;

    // desired orientation
    Eigen::Vector3d z_des = a_des / a_des.norm();
    Eigen::Vector3d x_des = y_c.cross(z_des) / y_c.cross(z_des).norm();
    Eigen::Vector3d y_des = z_des.cross(x_des);

    // initialize and fill rotation matrix
    Eigen::Matrix3d R_des;
    R_des.col(0) = x_des;
    R_des.col(1) = y_des;
    R_des.col(2) = z_des;

    // find euler angles
    Eigen::Vector3d euler = R_des.eulerAngles(0, 1, 2);

    // thrust command
    double c1 = a_des.transpose() * z_b;
    double c2 = (v.transpose() * (x_b + y_b));
    double c = c1 - params::k_h * c2 * c2;
    // publish commands
    Offboard::Attitude cmd{};
    cmd.roll_deg = euler(0);
    cmd.pitch_deg = euler(1);
    cmd.yaw_deg = euler(2);
    float thr = c / c_max;
    cmd.thrust_value = thr;
    // std::cout << "roll: " << euler(0) << " pitch: " << euler(1)
    //<< " yaw: " << euler(2) << " thrust: " << c << std::endl;
    std::cout << " thrust: " << thr << std::endl;
    offboard.set_attitude(cmd);
  }
  // std::cout << c_max << std::endl;
  //    float z;
  //    if (!params::SIM) {
  //      z = sub::mocap_msg.pose.position.z;
  //      mocap.listener->wait_for_data();
  //    } else {
  //      z = -telemetry.position_velocity_ned().position.down_m;
  //    }
  //    std::cout << "z= " << z << std::endl;
  //    p_ctrl = z_ref - z;
  //    i_ctrl += p_ctrl;
  //    float thrust = params::P * p_ctrl + params::I * i_ctrl;
  //    // saturation
  //    if (thrust > T_max) {
  //      thrust = T_max;
  //    }
  //    if (thrust < T_min) {
  //      thrust = T_min;
  //    }
  //    stay.thrust_value = thrust;
  //    hover_thrust = thrust;
  //    offboard.set_attitude(stay);

  // std::cout << hover_thrust << std::endl;

  // landing
  std::cout << "landing..." << std::endl;
  stay.thrust_value = 0.25;
  offboard.set_attitude(stay);
  sleep_for(seconds(5));
  stay.thrust_value = 0.0;
  offboard.set_attitude(stay);
  sleep_for(seconds(5));
  std::cout << "landed" << std::endl;

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

float constrain(float data, float under, float upper) {
  if (data > upper) {
    data = upper;
  }
  if (data < under) {
    data = under;
  }
  return data;
}
bool offb_ctrl_attitude_custom(
    mavsdk::Offboard &offboard, mavsdk::Telemetry &telemetry,
    DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> &mocap) {

  std::cout << "Starting Offboard attitude control\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::Attitude stay{};
  stay.roll_deg = 0.0f;
  stay.pitch_deg = 0.0f;
  stay.yaw_deg = 0.0f;
  stay.thrust_value = 0.3f;
  offboard.set_attitude(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard started\n";

  // LOGGING
  std::ofstream myLog;
  auto timenow =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string Date = ctime(&timenow);
  std::replace(Date.begin(), Date.end(), ' ', '_');
  Date.pop_back();
  myLog.open("log/Flight_" + Date + ".csv");
  std::cout << "Started logging to flight_" << Date << ".csv\n";

  // takeoff(ramp up approach)
  // Offboard::Attitude toff{};
  // toff.roll_deg = 0.0f;
  // toff.pitch_deg = 0.0f;
  // toff.yaw_deg = 0.0f;
  // toff.thrust_value = 0.5f;
  // offboard.set_attitude(toff);
  // sleep_for(seconds(3));

  float z_ref = 2.5;
  // control loop
  // std::chrono::steady_clock::time_point end =
  //     std::chrono::steady_clock::now() + seconds(params::T);
  float error = 0, i_error = 0, d_error = 0, last_error = 0;

  int loops = (params::T * 1000) / params::T_s;
  for (int t = 0; t < loops; t++) {
    // GET INFORMATION
    float z;
    float vz;
    if (!params::SIM) {
      // z = sub::mocap_msg.pose.position.z;
      // mocap.listener->wait_for_data();
    } else {
      // z = (-1.0) * telemetry.acceleration_frd().down_m_s2;
      z = (-1.0) * telemetry.position_velocity_ned().position.down_m;
      vz = (-1.0) * telemetry.position_velocity_ned().velocity.down_m_s;
    }
    // REFERENCE GENERATOR
    z_ref = 2.5 + std::sin((t * params::T_s) / 1000.0);

    // if (t * params::T_s < 5000) {
    //   z_ref = 2.5;
    // } else {
    //   z_ref = 2.5 + std::sin((t * params::T_s) / 2000.0);
    // }
    // POSITION CONTROLLER
    float z_err = z_ref - z;
    float v_ref = constrain(params::P_pos * z_err, -5, 5);
    std::cout << " z: " << z << "\t err: " << z_err << "\t v_ref: " << v_ref
              << std::endl;

    // VELOCITY CONTROLLER
    last_error = error;
    error = constrain(v_ref - vz, -5, 5);
    i_error = constrain(error + error * (params::T_s / 1000.0), -5, 5);
    d_error = constrain((error - last_error) * (1000.0 / params::T_s), -5, 5);
    float thrust =
        constrain(params::hover + params::P_vel * error +
                      i_error / params::I_vel + params::D_vel * d_error,
                  0, 1);
    // saturation

    std::cout << " v: " << vz << "\t pe: " << error << "\t ie: " << i_error
              << "\t de: " << d_error << "\t t: " << thrust << std::endl;

    // send cmd to quadcopter
    stay.thrust_value = thrust;
    offboard.set_attitude(stay);
    myLog << t * params::T_s << "," << z_ref << "," << z << "\n";
    sleep_for(milliseconds(int(params::T_s)));
  }

  // landing
  std::cout << "landing..." << std::endl;
  stay.thrust_value = 0.25;
  offboard.set_attitude(stay);
  sleep_for(seconds(5));
  stay.thrust_value = 0.0;
  offboard.set_attitude(stay);
  sleep_for(seconds(5));
  std::cout << "landed" << std::endl;

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

int main(int argc, char **argv) {

  // load yaml parameters
  set_parameters("app/parameters/params.yaml");
  // fast DDS
  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "mocap_subscriber");

  // Create publisher with msg type
  DDSSubscriber mocap_sub(idl_msg::MocapPubSubType(), &sub::mocap_msg,
                          "mocap_srl_quad", dp.participant());
  if (!params::SIM) {
    // activate mocap
    for (int i = 0; i < 10; i++) {
      mocap_sub.listener->wait_for_data();
      std::cout << "x= " << sub::mocap_msg.pose.position.x << std::endl;
    }
    // std::cout<<"starting fastDDS test"<<std::endl;
    // test(mocap_sub);
  }
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

  // while (!telemetry.health_all_ok()) {
  //     std::cout << "Waiting for system to be ready\n";
  //     sleep_for(seconds(1));
  // }
  std::cout << "System is ready\n";

  const auto arm_result = action.arm();
  if (arm_result != Action::Result::Success) {
    std::cerr << "Arming failed: " << arm_result << '\n';
    return 1;
  }
  std::cout << "Armed\n";

  // action.takeoff();
  // sleep_for(seconds(10));
  // offb_ctrl_acc(offboard);
  offb_ctrl_attitude_custom(offboard, telemetry, mocap_sub);

  const auto disarm_result = action.disarm();
  if (disarm_result != Action::Result::Success) {
    std::cerr << "Disarming failed: " << disarm_result << '\n';
    return 1;
  }
  std::cout << "Disarmed\n";

  sleep_for(seconds(3));
  std::cout << "Finished...\n";

  return 0;
}
