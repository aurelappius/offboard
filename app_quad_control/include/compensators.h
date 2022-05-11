#pragma once
#include <math.h>

const float quad_rotor_radius = 0.12;   // Quadcopter rotor radius [m]
const float quad_rotor_distance = 0.35; // Quadcopter rotor distance [m]
const float sanchez_constant = 2.0;     // constant defined in the sanchez publication
const float rho_air = 1.205;            // density of air [kg_m^3]
const float T_h = 1.516 * 9.81 / 4;     // hover thrust of one rotor[N]
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
// Appius Dynamic compensator
float AppiusDynamicCompensator(float thrust_ref, float z);
// Cheeseman Compensator with forward_speed
float CheesemanDynamicCompensator(float thrust_ref, float z, float v);
// Sanchez compensator with forward_speed
float KanDynamicCompensator(float thrust_ref, float z, float v);