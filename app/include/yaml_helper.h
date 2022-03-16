#pragma once

// namespace params
namespace params {
float P_vel_XY;
float I_vel_XY;
float D_vel_XY;
float P_vel_Z;
float I_vel_Z;
float D_vel_Z;
float P_pos_XY;
float P_pos_Z;
float T_log;
int T_s;
} // namespace params

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

// loads parameters
inline void set_parameters(const std::string setpoint_path) {

  // check if yaml file exists
  yaml_file_check(setpoint_path);
  // Load yaml file containing gains
  YAML::Node commands_yaml = YAML::LoadFile(setpoint_path);

  // Set parameters
  params::P_vel_XY = commands_yaml["P_vel_XY"].as<float>();
  params::I_vel_XY = commands_yaml["I_vel_XY"].as<float>();
  params::D_vel_XY = commands_yaml["D_vel_XY"].as<float>();
  params::P_pos_XY = commands_yaml["P_pos_XY"].as<float>();
  params::P_vel_Z = commands_yaml["P_vel_Z"].as<float>();
  params::I_vel_Z = commands_yaml["I_vel_Z"].as<float>();
  params::D_vel_Z = commands_yaml["D_vel_Z"].as<float>();
  params::P_pos_Z = commands_yaml["P_pos_Z"].as<float>();
  params::T_s = commands_yaml["T_s"].as<int>();
  params::T_log = commands_yaml["T_log"].as<float>();
}
