
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
