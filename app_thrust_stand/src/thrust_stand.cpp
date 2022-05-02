#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <future>
// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;

/* CONSTANTS */
const int motor_index = 0;
/* RPM, VOLTAGE, CURRENT */
int32_t rpm[4] = {0, 0, 0, 0};   // RPM [rotations per minute]
float voltage[4] = {0, 0, 0, 0}; // Voltage [Volts]
float current[4] = {0, 0, 0, 0}; // Current [Ampere]

// motor speed callback function
std::function<void(const mavlink_message_t &)> MotorSpeedCallback =
    [](const mavlink_message_t &raw_msg)
{
  mavlink_msg_esc_status_get_rpm(&raw_msg, &rpm[0]);
  mavlink_msg_esc_status_get_voltage(&raw_msg, &voltage[0]);
  mavlink_msg_esc_status_get_current(&raw_msg, &current[0]);

  // for (int i = 0; i < 4; i++)
  // {
  //   std::cout << "m " << i + 1 << ": rpm: \t" << rpm[i] << "\t volt: \t" << voltage[i] << "\t amps: \t" << current[i] << std::endl;
  // }
};

int main(int argc, char **argv)
{
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  std::string Name = "temp";
  myLog.open("log/THRUST_STAND_" + Name + ".csv");
  std::cout << "Started logging to log/" << Name << ".csv\n";

  /* INITIALIZE MAVSDK */
  if (argc != 2)
  {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  auto action = Action{system};              // for arming / disarming etc
  auto offboard = Offboard{system};          // for offboard control
  auto telemetry = Telemetry{system};        // for telemetry services
  auto mavlink = MavlinkPassthrough{system}; // for mavlink passtrough

  std::cout << "System is ready\n";

  /* MAVLINK MOTOR SPEED MESSAGES */
  std::cout << "subscribe to motor speeds" << std::endl;
  mavlink.subscribe_message_async(291, MotorSpeedCallback);

  /* ARM QUADCOPTER */
  const auto arm_result = action.arm();

  /* STARTING OFFBOARD */
  // construct actuator control group message right to evade Segmenation faults.
  Offboard::ActuatorControlGroup grp;
  for (int i = 0; i < 8; i++)
  {
    grp.controls.push_back(0.0);
  }
  Offboard::ActuatorControl act_cmd{};
  act_cmd.groups.push_back(grp);
  act_cmd.groups.push_back(grp);
  offboard.set_actuator_control(act_cmd);
  // start offboard
  Offboard::Result offboard_result = offboard.start();

  /* GO THROUGHT ALL THROTTLES FROM 0 TO 1 */
  for (float throttle = 0.15; throttle <= 1.05; throttle += 0.05)
  {
    // send thrust command
    // att_cmd.thrust_value = throttle;
    // offboard.set_attitude(att_cmd);
    // act_cmd.groups.at(0).controls.at(3) = throttle;
    std::cout << "now setting throttle = " << throttle << std::endl;
    act_cmd.groups.at(0).controls.at(3) = throttle;
    offboard.set_actuator_control(act_cmd);
    //

    // wait for thrust to settle
    std::this_thread::sleep_for(milliseconds(2500)); // 50Hz

    /* LOGGING (get average rpm over one thrust step*/
    int rpm_total = 0;
    float volt_total = 0;
    float amp_total = 0;
    for (int i = 0; i < 300; i++)
    {
      rpm_total += rpm[motor_index];
      amp_total += current[motor_index];
      volt_total += voltage[motor_index];
      myLog << throttle << "," << rpm[motor_index] << "," << current[motor_index] << "," << voltage[motor_index] << "\n";
      std::this_thread::sleep_for(milliseconds(20)); // 50Hz
    }
    float rpm_avg = ((float)rpm_total) / 300.0;
    float amp_avg = (amp_total) / 300.0;
    float volt_avg = (volt_total) / 300.0;

    std::cout << "throttle:\t" << throttle << "\t rpm: \t" << rpm_avg << "\t volt: \t" << volt_avg << "\t amps: \t" << amp_avg << std::endl;
  }

  /* DISARM QUADCOPTER */
  std::this_thread::sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';
  myLog.close();
  return 0;
}
