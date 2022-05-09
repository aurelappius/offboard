#include "include_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/* RPM, VOLTAGE, CURRENT */
int32_t rpm[4] = {0, 0, 0, 0};   // RPM [rotations per minute]
float voltage[4] = {0, 0, 0, 0}; // Voltage [Volts]
float current[4] = {0, 0, 0, 0}; // Current [Ampere]

/* FUNCTION DECLARATIONS */
// thrust-throttle relation (linear)
float thrust_to_throttle(float thrust)
{
  if (thrust > 4 * params::max_motor_thrust)
  {
    return 1;
  }
  if (thrust < 0)
  {
    return 0;
  }
  return (0.02394 * thrust + 0.1644);
}

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

  /* LOAD YAML PARAMETERS */
  set_parameters("app_quad_control/parameters/params.yaml");

  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  std::string Name = "STATIC_DATA";
  myLog.open("log/" + Name + ".csv");
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

  /* LOAD PLUGINS */
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

  for (int i = 0;; i++)
  { // control loop at 50Hz
    // indices -> real-time
    t = float(i * params::T_s) / 1000.0;
    // std::cout<<"offboarding"<<std::endl;
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
    //          vel(2)
    //           << std::endl;

    /* TRAJECTORY GENERATION */
    // stepResponse(t, pos, pos_ref, yaw_ref);
    verticalSpeedController(t, pos, pos_ref, yaw_ref);
    /* POSITION CONTROLLER */
    // proportional position error
    pos_p_error = pos_ref - pos;
    // std::cout<<"pos:"<<pos_p_error<<std::endl;
    //  desired velocity
    vel_ref(0) = params::P_pos_XY * pos_p_error(0);
    vel_ref(1) = params::P_pos_XY * pos_p_error(1);
    vel_ref(2) = params::P_pos_Z * pos_p_error(2); // different gain for Z-error

    // check maximum velocities and constrain.
    if (vel_ref(0) > params::max_vel_XY)
    {
      vel_ref(0) = params::max_vel_XY;
    }
    if (vel_ref(0) < -params::max_vel_XY)
    {
      vel_ref(0) = -params::max_vel_XY;
    }
    if (vel_ref(1) > params::max_vel_XY)
    {
      vel_ref(1) = params::max_vel_XY;
    }
    if (vel_ref(1) < -params::max_vel_XY)
    {
      vel_ref(1) = -params::max_vel_XY;
    }
    if (vel_ref(2) > params::max_vel_Z_UP)
    {
      vel_ref(2) = params::max_vel_Z_UP;
    }
    if (vel_ref(2) < -params::max_vel_Z_DOWN)
    {
      vel_ref(2) = -params::max_vel_Z_DOWN;
    }

    /* VELOCITY CONTROLLER */
    // last proportional velocity error
    vel_p_error_last = vel_p_error;
    // std::cout<<"vel:"<<vel_p_error<<std::endl;
    //  proportional velocity error
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
    acc_ref(2) = acc_ref(2); // - params::g;
                             // std::cout<<"acc:"<<acc_ref(2)<<std::endl;
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

    float thrust_ref = (acc_proj_z_b)*params::quadcopter_mass; // F=M*a

    // if (t > 15)
    // {
    //   thrust_ref = NobahariCompensator(thrust_ref, pos(2)); // GE compensator
    // }

    float throttle_ref = thrust_to_throttle(thrust_ref);
    // std::cout<<throttle_ref<<std::endl;
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
    // std::cout << thrust_ref << std::endl;
    // std::cout << throttle_ref << std::endl;
    offboard.set_attitude(att_cmd);

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
    // if (t > 20 && t <= 55)
    // { //(t > params::T_log) { // possibility to wait for
    // transients to fade away
    // if (telemetry.actuator_control_target().controls.size() != 0)
    // {
    myLog << t << ","
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
          //  << telemetry.actuator_control_target().controls.at(0) << ","
          // << telemetry.actuator_control_target().controls.at(1) << ","
          //  << telemetry.actuator_control_target().controls.at(2) << ","
          //  << telemetry.actuator_control_target().controls.at(3) << ","
          << rpm[0] << ","
          << rpm[1] << ","
          << rpm[2] << ","
          << rpm[3] << ","
          << voltage[0] << ","
          << voltage[1] << ","
          << voltage[2] << ","
          << voltage[3] << ","
          << current[0] << ","
          << current[1] << ","
          << current[2] << ","
          << current[3] << "\n";
    // }
    // }

    /* SLEEP */
    sleep_for(milliseconds(params::T_s)); // 50Hz
  }

  /* DISARM QUADCOPTER */
  sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';

  return 0;
}

// CODE TO DEBUG TRAJECTORIES
/*

int main(int argc, char **argv)
{
  float t = 0;
  int T_s = 20;
  Eigen::Vector3f pos_ref(0, 0, 0);
  Eigen::Vector3f pos(0, 0, 0);
  float yaw_ref = 0;
  for (int i = 0;; i++)
  {
    t = float(i * T_s) / 1000.0;
    // stepResponse(t, pos, pos_ref, yaw_ref); //-> works
    // circleStepResponse(t, pos, pos_ref, yaw_ref); //-> works
    // staticDataCollection(t, pos, pos_ref, yaw_ref); //-> works
    // verticalSpeedDataCollection(t, pos, pos_ref, yaw_ref); //->works
    std::cout
        << "t: \t" << t << "\t x: \t" << pos_ref(0) << "\t y: \t" << pos_ref(1) << "\t z: \t" << pos_ref(2) << std::endl;
    sleep_for(milliseconds(T_s)); // 50Hz
  }
}

*/
