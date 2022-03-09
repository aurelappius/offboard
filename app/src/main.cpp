#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <queue>
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
  params::v = commands_yaml["v"].as<float>();
  params::h_0 = commands_yaml["h_0"].as<float>();
  params::freq = commands_yaml["freq"].as<float>();
  params::amp = commands_yaml["amp"].as<float>();
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

bool offb_pos_ctrl_ned(
    mavsdk::Offboard &offboard, mavsdk::Telemetry &telemetry,
    DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> &mocap,
    std::ofstream &myLog) {
  std::cout << "Starting Offboard position control in NED coordinates\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::PositionNedYaw msg;
  msg.down_m = -params::h_0;
  msg.east_m = 1.5;
  msg.north_m = -2;
  msg.yaw_deg = 0;
  offboard.set_position_ned(msg);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  // go to start position
  offboard.set_position_ned(msg);
  // wait until takeoff alitutde is reached
  std::cout << "going to starting position" << std::endl;
  // while (true) {
  //   if (!params::SIM) {
  //     if (sub::mocap_msg.pose.position.x > -2.1 &&
  //         sub::mocap_msg.pose.position.x < -1.9) {
  //       break;
  //     }
  //   } else {
  //     if (telemetry.position_velocity_ned().position.north_m > -2.1 &&
  //         telemetry.position_velocity_ned().position.north_m < -1.9 &&
  //         telemetry.position_velocity_ned().velocity.north_m_s < 0.01) {
  //       break;
  //     }
  //   }
  //   sleep_for(milliseconds(250));
  // }
  sleep_for(seconds(5));
  std::cout << "starting experiment" << std::endl;

  // Control loop
  int loops;
  if (params::v == 0) {
    loops = (params::T * 1000.0) / (float(params::T_s));
  } else {
    loops = (4 * 1000.0) / (params::v * float(params::T_s));
  }
  for (int t = 0; t < loops; t++) {
    float z_ref;

    float t_real = float(t * params::T_s) / 1000.0;
    if (t_real < 5) {
      z_ref = params::h_0;
    }
    if (t_real > 5) {
      z_ref = params::h_0 - 1.0;
    }

    float x_ref = -2 + params::v * t_real;
    float y_ref = 1.5;
    // float z_ref =
    //     params::h_0 - params::amp * std::sin(t_real * 2 * M_PI *
    //     params::freq);
    float z;

    msg.north_m = x_ref;
    msg.east_m = y_ref;
    msg.down_m = -z_ref;
    offboard.set_position_ned(msg);

    /*
        std::vector<float> data_from_quad =
            telemetry.actuator_control_target().controls;

        data_from_quad.at(3) = 0.75 * data_from_quad.at(3);

        mavsdk::Offboard::ActuatorControlGroup actuatorCtrlGroup1;
        actuatorCtrlGroup1.controls = data_from_quad;

        std::vector<float> empty{0, 0, 0, 0, 0, 0, 0, 0};
        mavsdk::Offboard::ActuatorControlGroup actuatorCtrlGroup2;
        actuatorCtrlGroup2.controls = empty;

        std::vector<mavsdk::Offboard::ActuatorControlGroup>
       actuatorCtrlGroup_vec;

        actuatorCtrlGroup_vec.push_back(actuatorCtrlGroup1);
        actuatorCtrlGroup_vec.push_back(actuatorCtrlGroup2);

        mavsdk::Offboard::ActuatorControl actuatorCtrl;

        actuatorCtrl.groups = actuatorCtrlGroup_vec;
        // actuatorCtrl.groups = actuatorCtrl.groups.at(0);

        offboard.set_actuator_control(actuatorCtrl);
    */
    // cout time, z_ref, z, thrust_cmd
    // std::cout << "t: " << t * params::T_s << "\t z_ref: " << z_ref
    //           << "\t z: " << z
    //           << "\t t: " <<
    //           telemetry.actuator_control_target().controls.at(3)
    //           << std::endl;

    // log time, xyz_ref, xyz, rpy, control rpyt
    if (!params::SIM) {
      myLog << t * params::T_s << "," << x_ref << "," << y_ref << "," << z_ref
            << "," << sub::mocap_msg.pose.position.x << ","
            << sub::mocap_msg.pose.position.y << ","
            << sub::mocap_msg.pose.position.z << ","
            << sub::mocap_msg.pose.orientation_euler.roll << ","
            << sub::mocap_msg.pose.orientation_euler.pitch << ","
            << sub::mocap_msg.pose.orientation_euler.yaw << "\n";
      // << telemetry.actuator_control_target().controls.at(0) << ","
      // << telemetry.actuator_control_target().controls.at(1) << ","
      // << telemetry.actuator_control_target().controls.at(2) << ","
      // << telemetry.actuator_control_target().controls.at(3) << "\n";
    } else {
      myLog << t * params::T_s << "," << x_ref << "," << y_ref << "," << z_ref
            << "," << telemetry.position_velocity_ned().position.north_m << ","
            << telemetry.position_velocity_ned().position.east_m << ","
            << -telemetry.position_velocity_ned().position.down_m << ","
            << telemetry.position_velocity_ned().velocity.north_m_s << ","
            << telemetry.position_velocity_ned().velocity.east_m_s << ","
            << -telemetry.position_velocity_ned().velocity.down_m_s << ","
            << telemetry.attitude_euler().roll_deg << ","
            << telemetry.attitude_euler().pitch_deg << ","
            << telemetry.attitude_euler().yaw_deg << ","
            << telemetry.actuator_control_target().controls.at(0) << ","
            << telemetry.actuator_control_target().controls.at(1) << ","
            << telemetry.actuator_control_target().controls.at(2) << ","
            << telemetry.actuator_control_target().controls.at(3) << "\n";
    }
    sleep_for(milliseconds(int(params::T_s)));
  }

  std::cout << "experiment finished" << std::endl;
  sleep_for(seconds(2));

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

float constrain(float data, float under, float upper) {
  if (data > upper) {
    data = upper;
  }
  if (data < under) {
    data = under;
  }
  return data;
}

int main(int argc, char **argv) {

  // load yaml parameters
  set_parameters("app/parameters/params.yaml");

  // FASTDDS
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
    std::cout << "initialized mocap" << std::endl;
  }

  // LOGGING

  std::ofstream myLog;

  std::string Name_0 = "flight";
  std::string suffix = "_nr_0";
  std::string Name_h0 = "_h0_";
  std::string val_h0 = std::to_string(params::h_0);
  val_h0.erase(val_h0.find_last_not_of('0') + 2, std::string::npos);
  std::string Name_v = "_v_";
  std::string val_v = std::to_string(params::v);
  val_v.erase(val_v.find_last_not_of('0') + 2, std::string::npos);
  std::string Name_freq = "_freq_";
  std::string val_freq = std::to_string(params::freq);
  val_freq.erase(val_freq.find_last_not_of('0') + 2, std::string::npos);
  std::string Name_amp = "_amp_";
  std::string val_amp = std::to_string(params::amp);
  val_amp.erase(val_amp.find_last_not_of('0') + 2, std::string::npos);
  std::string Name_sim;
  if (params::SIM == true) {
    Name_sim = "_SIM";
  } else {
    Name_sim = "_REAL";
  }
  std::string Name = Name_0 + Name_h0 + val_h0 + Name_v + val_v + Name_freq +
                     val_freq + Name_amp + val_amp + Name_sim + suffix;

  // check if file already exists
  for (int i = 0;; i++) {
    Name.pop_back();
    Name = Name + std::to_string(i);
    std::filesystem::path f{"log/" + Name + ".csv"};
    if (!(std::filesystem::exists(f))) {
      break;
    }
  }

  myLog.open("log/" + Name + ".csv");

  std::cout << "Started logging to log/" << Name << ".csv\n";

  // contdown [for testing]
  for (int i = 3; i > 0; i--) {
    std::cout << "T minus " << i << std::endl;
    sleep_for(seconds(1));
  }
  std::cout << "LIFTOFF!" << std::endl;

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

  action.takeoff();

  // wait until takeoff alitutde is reached
  while (true) {
    if (!params::SIM) {
      if (sub::mocap_msg.pose.position.z > 1.4) {
        break;
      }
    } else {
      if (-telemetry.position_velocity_ned().position.down_m > 1.4) {
        break;
      }
    }
    sleep_for(milliseconds(250));
  }

  sleep_for(seconds(3));

  offb_pos_ctrl_ned(offboard, telemetry, mocap_sub, myLog);

  // for (int a = 0; a < 100; a++) {
  //   Telemetry::ActuatorControlTarget hoho =
  //   telemetry.actuator_control_target(); std::cout << "size: " <<
  //   telemetry.actuator_control_target().controls.size()
  //             << std::endl;
  //   for (int i = 0; i < hoho.controls.size(); i++) {
  //     std::cout << "eintrag " << i << ": " << hoho.controls.at(i) <<
  //     std::endl;
  //   }
  //   sleep_for(milliseconds(100));
  // }

  action.land();

  // wait until landed
  while (true) {
    if (!params::SIM) {
      if (sub::mocap_msg.pose.position.z < 0.1) {
        break;
      }
    } else {
      if (-telemetry.position_velocity_ned().velocity.down_m_s < 0.01 &&
          -telemetry.position_velocity_ned().position.down_m < 0.1) {
        break;
      }
    }
    sleep_for(milliseconds(250));
  }
  sleep_for(seconds(5));
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
