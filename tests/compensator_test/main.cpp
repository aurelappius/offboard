#include "gnuplot-iostream.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

/* CONSTANTS */
const float g = 9.81;                   // gravitational acceleration [m_s2]
const float quadcopter_mass = 1.5;      // Quadcopter mass [kg]
const float max_thrust = 4 * 8.9764;    // maximal thrust [N]
const float quad_rotor_radius = 0.12;   // Quadcopter rotor radius [m]
const float quad_rotor_distance = 0.35; // Quadcopter rotor distance [m]
const float Sanchez_constant = 2;       // constant for sanchez compensator

typedef std::pair<double, double> Point;

// Cheeseman compensator
float CheesemanCompensator(float throttle_ref, float z)
{
    return throttle_ref * (1.0 - std::pow((quad_rotor_radius / (4 * z)), 2));
}
// Nobahari compensator (R_eq = 2.5*R)
float NobahariCompensator(float throttle_ref, float z)
{
    return throttle_ref *
           (1.0 - std::pow((2.5 * quad_rotor_radius / (4 * z)), 2));
}

// Hayden compensator
float HaydenCompensator(float throttle_ref, float z)
{
    return throttle_ref * std::pow(0.9926 + 0.03794 * 4 * quad_rotor_radius *
                                                quad_rotor_radius / (z * z),
                                   (-2.0 / 3.0));
}

// Sanchez compensator
float SanchezCompensator(float throttle_ref, float z)
{
    float d = quad_rotor_distance;
    float R = quad_rotor_radius;
    return throttle_ref *
           (1 - std::pow(R / (4 * z), 2) -
            R * R * (z / std::pow(std::pow(d * d + 4 * z * z, 3), 0.5)) -
            0.5 * R * R *
                (z / std::pow(std::pow(2 * d * d + 4 * z * z, 3), 0.5) *
                 Sanchez_constant));
}

// real time plot
void real_time_plot()
{
    Gnuplot gp;

    gp << "set terminal wxt size 800, 400\n";
    gp << "set xrange [0:2]" << std::endl;
    gp << "set yrange [0:2]" << std::endl;

    std::vector<std::pair<float, float>> Cheeseman;
    std::vector<std::pair<float, float>> Hayden;
    std::vector<std::pair<float, float>> Nobahari;
    std::vector<std::pair<float, float>> Sanchez;

    for (float z_it = 2.0; z_it > 0.02; z_it -= 0.01)
    {

        Cheeseman.push_back(std::make_pair(z_it, CheesemanCompensator(1, z_it)));
        Hayden.push_back(std::make_pair(z_it, HaydenCompensator(1, z_it) + 0.25));
        Nobahari.push_back(
            std::make_pair(z_it, NobahariCompensator(1, z_it) + 0.5));
        Sanchez.push_back(std::make_pair(z_it, SanchezCompensator(1, z_it) + 0.75));

        gp << "plot";
        gp << " '-' with lines title 'Cheeseman',";
        gp << " '-' with lines title 'Hayden',";
        gp << " '-' with lines title 'Nobahari',";
        gp << " '-' with lines title 'Sanchez'" << std::endl;
        gp.send1d(Cheeseman);
        gp.send1d(Hayden);
        gp.send1d(Nobahari);
        gp.send1d(Sanchez);
        // gp.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
void static_plot()
{
    Gnuplot gp;
    // variables
    std::vector<std::pair<float, float>> Cheeseman;
    std::vector<std::pair<float, float>> Hayden;
    std::vector<std::pair<float, float>> Nobahari;
    std::vector<std::pair<float, float>> Sanchez;

    for (float z_it = 2.0; z_it > 0.02; z_it -= 0.001)
    {
        Cheeseman.push_back(std::make_pair(z_it, CheesemanCompensator(1, z_it)));
        Hayden.push_back(std::make_pair(z_it, HaydenCompensator(1, z_it)));
        Nobahari.push_back(std::make_pair(z_it, NobahariCompensator(1, z_it)));
        Sanchez.push_back(std::make_pair(z_it, SanchezCompensator(1, z_it)));
        // std::cout << "=======z: " << z << " =======" << std::endl;
        // std::cout << "Cheeseman:\t" << CheesemanCompensator(1.0, z) <<std::endl;
        // std::cout << "Hayden:  \t" << HaydenCompensator(1.0, z) << std::endl;
        // std::cout << "Nobahari:\t" << NobahariCompensator(1.0, z) <<std::endl;
        // std::cout << "Sanchez:\t" << SanchezCompensator(1.0, z) << std::endl;
    }
    gp << "set xrange [0:1]" << std::endl;
    gp << "set yrange [0:2]" << std::endl;
    gp << "set xlabel 'z [m]'" << std::endl;
    gp << "set ylabel 'T_{oge}/T_{ige}'" << std::endl;
    gp << "plot" << gp.file1d(Cheeseman) << "with lines title 'Cheeseman',";
    gp << gp.file1d(Hayden) << "with lines title 'Hayden',";
    gp << gp.file1d(Nobahari) << "with lines title 'Nobahari',";
    gp << gp.file1d(Sanchez) << "with lines title 'Sanchez'," << std::endl;
}
int main()
{
    real_time_plot();
    return 0;
}
