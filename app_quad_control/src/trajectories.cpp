#include "trajectories.h"
/* BENCHMARK TRAJECTORIES */
// vertical step response from 1.20 m to 0.20 m
void stepResponse(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    const float h_0 = 1.2; // height before step [m]
    const float h_1 = 0.2; // height after step [m]

    if (t > 0 && t <= 20)
    { // takeoff
        pos_ref(0) = 0;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // step response
    if (t > 20 && t <= 55)
    {
        pos_ref(0) = 0;
        pos_ref(1) = 0;
        pos_ref(2) = h_1;
        yaw_ref = 0.0;
    }
    if (t > 55)
    { // land
        std::cout << "landing now" << std::endl;
        pos_ref(0) = pos(0);
        pos_ref(1) = pos(1);
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// vertical step response from 1.20 m to 0.20 m with forward speed
void circleStepResponse(float t, Eigen::Vector3f &pos,
                        Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // constants
    const float h_0 = 1.2;                                // height before step [m]
    const float h_1 = 0.2;                                // height after step [m]
    const float speed = 0.5;                              // horizontal speed [m_s]
    const float circle_radius = 1.0;                      // radius of flown circle [m]
    const float angular_velocity = speed / circle_radius; // angular velocity [1_s]
    const float circle_center_x = 1.0;                    // center of circle [m]
    const float circle_center_y = 1.0;                    // center of circle [m]

    // takeoff and go to start
    if (t > 0 && t <= 20)
    {
        pos_ref(0) = circle_center_x + circle_radius * std::cos(0);
        pos_ref(1) = circle_center_y + circle_radius * std::sin(0);
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // start flying circles
    if (t > 20 && t <= 40)
    {
        pos_ref(0) = circle_center_x + circle_radius * std::cos(angular_velocity * (t - 20));
        pos_ref(1) = circle_center_y + circle_radius * std::sin(angular_velocity * (t - 20));
        pos_ref(2) = h_0;
        yaw_ref = angular_velocity * (t - 20);
    }
    // step down while flying circles
    if (t > 40 && t <= 60)
    {
        pos_ref(0) = circle_center_x + circle_radius * std::cos(angular_velocity * (t - 20));
        pos_ref(1) = circle_center_y + circle_radius * std::sin(angular_velocity * (t - 20));
        pos_ref(2) = h_1;
        yaw_ref = angular_velocity * (t - 20);
    }

    if (t > 60)
    { // land
        std::cout << "landing now" << std::endl;
        pos_ref(0) = pos(0);
        pos_ref(1) = pos(1);
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// oscillations
void oscillations(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // TODO
}

/* DATA COLLECTION TRAJECTORIES */
// quasi static flying for simple GE Model
void staticDataCollection(float t, Eigen::Vector3f &pos,
                          Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // constants
    const float h_0 = 1.2;                    // height before step [m]
    const float h_1 = 0.1;                    // height after step [m]
    const float h_step = 0.05;                // step size [m]
    const float t_step = 5;                   // time for a step [s]
    const int n_steps = (h_0 - h_1) / h_step; // number of steps []
    const float t_down = n_steps * t_step;    // time for a descent / climb [s]
    const float x = 0.5;                      // x-coordinate [m]
    const float y = 0.5;                      // y-coordinate [m]

    // find OGE thrust
    if (t > 0 && t <= 30)
    {
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // go down in steps of h_step
    if (t > 30 && t <= 30 + t_down)
    {
        int i = float((t - 30.0) / t_down * n_steps);
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = h_0 - i * h_step;
        yaw_ref = 0.0;
    }
    // go up in steps of h_step
    if (t > 30 + t_down && t <= 30 + 2 * t_down)
    {
        int i = float((t - 30 - t_down) / t_down * n_steps);
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = h_1 + i * h_step;
        yaw_ref = 0.0;
    }
    // hover a little
    if (t > 30 + 2 * t_down && t <= 30 + 2 * t_down + 10)
    {
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // land
    if (t > 30 + 2 * t_down + 10)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// vertical speed flying for more advanced GE Model
void verticalSpeedDataCollection(float t, Eigen::Vector3f &pos,
                                 Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    const float offset = 1.25; // vertical offset from IGE to OGE [m]
    const float T_seq = 5;     // time for a certain altitude command [s]
    const float x = 0.5;       // x-coordinate [m]
    const float y = 0.5;       // y-coordinate [m]

    // Taking off
    if (t > 0 && t <= 20)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = 2;
        yaw_ref = 0.0;
    }
    // flying in Low altitude (IGE)
    if (t > 20 && t < 70)
    {
        pos_ref(1) = x;
        pos_ref(2) = y;
        yaw_ref = 0.0;
        int i = (t - 20.0) / T_seq;

        switch (i)
        {
        case 0:
            pos_ref(2) = 1.1 + offset;
            break;
        case 2:
            pos_ref(2) = 0.9 + offset;
            break;
        case 4:
            pos_ref(2) = 0.7 + offset;
            break;
        case 6:
            pos_ref(2) = 0.5 + offset;
            break;
        case 8:
            pos_ref(2) = 0.3 + offset;
            break;
        default:
            pos_ref(2) = 0.1 + offset;
            break;
        }
    }
    // flying in Low altitude (IGE)
    if (t > 70 && t < 120)
    {
        pos_ref(1) = x;
        pos_ref(2) = y;
        yaw_ref = 0.0;
        int i = (t - 20.0) / T_seq;

        switch (i)
        {
        case 0:
            pos_ref(2) = 1.1;
            break;
        case 2:
            pos_ref(2) = 0.9;
            break;
        case 4:
            pos_ref(2) = 0.7;
            break;
        case 6:
            pos_ref(2) = 0.5;
            break;
        case 8:
            pos_ref(2) = 0.3;
            break;
        default:
            pos_ref(2) = 0.1;
            break;
        }
    }
    // flying in High altitude (OGE)

    if (t > 120)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// omnidirectional flying for most advanced GE Model
void allSpeedDataCollection(float t, Eigen::Vector3f &pos,
                            Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // TODO
}
