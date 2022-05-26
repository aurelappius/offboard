#pragma once
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

// Cheeseman Compensator
float CheesemanCompensator(float thrust_ref, float z);
// Nobahari compensator (R_eq: 2.5*R)
float NobahariCompensator(float thrust_ref, float z);
// Hayden compensator
float HaydenCompensator(float thrust_ref, float z);
// Sanchez compensator
float SanchezCompensator(float thrust_ref, float z);
// Appius Static compensator
float AppiusCompensator(float thrust_ref, float z);
// Cheeseman Compensator with forward_speed
float CheesemanDynamicCompensator(float thrust_ref, float z, float v);
// Kan compensator with forward_speed
float KanDynamicCompensator(float thrust_ref, float z, float v);