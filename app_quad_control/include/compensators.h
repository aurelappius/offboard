#pragma once
#include <math.h>

const float quad_rotor_radius = 0.12;   // Quadcopter rotor radius [m]
const float quad_rotor_distance = 0.35; // Quadcopter rotor distance [m]
const float sanchez_constant = 2.0;     // constant defined in the sanchet publication
// Cheeseman Compensator
float CheesemanCompensator(float thrust_ref, float z);
// Nobahari compensator (R_eq: 2.5*R)
float NobahariCompensator(float thrust_ref, float z);
// Hayden compensator
float HaydenCompensator(float thrust_ref, float z);
// Sanchez compensator
float SanchezCompensator(float thrust_ref, float z);
// Appius Static compensator
float AppiusStaticCompensator(float thrust_ref, float z);