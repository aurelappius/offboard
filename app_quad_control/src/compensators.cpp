#include "compensators.h"
float CheesemanCompensator(float thrust_ref, float z)
{
    return thrust_ref * (1.0 - std::pow((quad_rotor_radius / (4 * z)), 2));
}
// Nobahari compensator (R_eq = 2.5*R)
float NobahariCompensator(float thrust_ref, float z)
{
    return thrust_ref *
           (1.0 - std::pow((2.5 * quad_rotor_radius / (4 * z)), 2));
}

// Hayden compensator
float HaydenCompensator(float thrust_ref, float z)
{
    return thrust_ref * std::pow(0.9926 + 0.03794 * 4 * quad_rotor_radius *
                                              quad_rotor_radius / (z * z),
                                 (-2.0 / 3.0));
}

// Sanchez compensator
float SanchezCompensator(float thrust_ref, float z)
{
    float d = quad_rotor_distance;
    float R = quad_rotor_radius;
    return thrust_ref *
           (1 - std::pow(R / (4 * z), 2) -
            R * R * (z / std::pow(std::pow(d * d + 4 * z * z, 3), 0.5)) -
            0.5 * R * R *
                (z / std::pow(std::pow(2 * d * d + 4 * z * z, 3), 0.5) *
                 sanchez_constant));
}

// Appius Static Compensator
float AppiusStaticCompensator(float thrust_ref, float z)
{
    return thrust_ref / (0.11413538 * std::exp(-5.38792044 * z) + 1.0275278);
}
