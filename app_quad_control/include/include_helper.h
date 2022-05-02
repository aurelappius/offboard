#pragma once

// general dependencies
#include <Eigen/Dense>
#include <cmath>
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

// yaml
#include <yaml-cpp/yaml.h>

// helpers
#include "mavsdk_helper.h"
#include "yaml_helper.h"

// trajectories and compensators
#include "trajectories.h"
#include "compensators.h"
