#include <chrono>
#include <cmath>
#include <filesystem>
#include <future>
#include <iostream>
#include <thread>

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
  params::P = commands_yaml["p_gain"].as<float>();
  params::I = commands_yaml["i_gain"].as<float>();
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
  takeoff.down_m_s2 = -5;
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

bool offb_ctrl_attitude(
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

  // params
  float hover_thrust = 0;
  float amp = 0.2;

  // keep altitidue
  float z_ref = 2.0;
  float i_ctrl = 0;
  float p_ctrl = 0;
  float T_max = 0.6;
  float T_min = 0.0;

  int T = 30;
  std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now() + seconds(T);

  while (std::chrono::steady_clock::now() < end) {
    float z;
    if (!params::SIM) {
      z = sub::mocap_msg.pose.position.z;
      mocap.listener->wait_for_data();
    } else {
      z = -telemetry.position_velocity_ned().position.down_m;
    }
    std::cout << "z= " << z << std::endl;
    p_ctrl = z_ref - z;
    i_ctrl += p_ctrl;
    float thrust = params::P * p_ctrl + params::I * i_ctrl;
    // saturation
    if (thrust > T_max) {
      thrust = T_max;
    }
    if (thrust < T_min) {
      thrust = T_min;
    }
    stay.thrust_value = thrust;
    hover_thrust = thrust;
    offboard.set_attitude(stay);
  }

  std::cout << hover_thrust << std::endl;

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

  // offb_ctrl_acc(offboard);
  offb_ctrl_attitude(offboard, telemetry, mocap_sub);

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
