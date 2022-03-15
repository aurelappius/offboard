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

  const auto takeoff_result = action.takeoff();
  std::cerr << "Takeoff Result: " << takeoff_result << '\n';

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::VelocityNedYaw vel_cmd{};
  offboard.set_velocity_ned(vel_cmd);

  Offboard::Result offboard_result = offboard.start();
  std::cerr << "Offboard Result: " << offboard_result << '\n';

  // variable definitions
  std::vector<float> p_ref{0, 0, 0};
  std::vector<float> p{0, 0, 0};
  std::vector<float> p_error{0, 0, 0};
  std::vector<float> v_ref{0, 0, 0};
  float t = 0;

  for (int i = 0;; i++) {
    // time
    t = float(i * params::T_s) / 1000.0;
    // trajectory
    p_ref.at(0) = sin(t / 5);
    p_ref.at(1) = cos(t / 5);
    p_ref.at(2) = 2;

    // current state
    p.at(0) = telemetry.position_velocity_ned().position.north_m;
    p.at(1) = telemetry.position_velocity_ned().position.east_m;
    p.at(2) = -telemetry.position_velocity_ned().position.down_m;

    // error
    p_error.at(0) = p_ref.at(0) - p.at(0);
    p_error.at(1) = p_ref.at(1) - p.at(1);
    p_error.at(2) = p_ref.at(2) - p.at(2);

    // desired velocity
    v_ref.at(0) = params::P_pos * p_error.at(0);
    v_ref.at(1) = params::P_pos * p_error.at(1);
    v_ref.at(2) = params::P_pos * p_error.at(2);

    // send offboard command
    vel_cmd.north_m_s = v_ref.at(0);
    vel_cmd.east_m_s = v_ref.at(1);
    vel_cmd.down_m_s = -v_ref.at(2);
    offboard.set_velocity_ned(vel_cmd);

    // logging (t,p_ref,p)
    if (t > 10) {
      myLog << t << "," << p_ref.at(0) << "," << p_ref.at(1) << ","
            << p_ref.at(2) << "," << p.at(0) << "," << p.at(1) << "," << p.at(2)
            << "\n";
    }
    sleep_for(milliseconds(params::T_s)); // 50Hz
  }

  sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';

  return 0;
}
