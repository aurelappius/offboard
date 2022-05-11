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
    const float speed = 0.2;                              // horizontal speed [m_s]
    const float circle_radius = 0.5;                      // radius of flown circle [m]
    const float angular_velocity = speed / circle_radius; // angular velocity [1_s]
    const float circle_center_x = 1.5;                    // center of circle [m]
    const float circle_center_y = 1.5;                    // center of circle [m]

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
        pos_ref(0) = circle_center_x + circle_radius * std::cos(angular_velocity * (t - 20) * (2 * M_PI));
        pos_ref(1) = circle_center_y + circle_radius * std::sin(angular_velocity * (t - 20) * (2 * M_PI));
        pos_ref(2) = h_0;
        yaw_ref = angular_velocity * (t - 20);
    }
    // step down while flying circles
    if (t > 40 && t <= 60)
    {
        pos_ref(0) = circle_center_x + circle_radius * std::cos(angular_velocity * (t - 20) * (2 * M_PI));
        pos_ref(1) = circle_center_y + circle_radius * std::sin(angular_velocity * (t - 20) * (2 * M_PI));
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

void verticalSpeedController(float t, Eigen::Vector3f &pos,
                             Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // constants
    const float h_0 = 0.9; // height before step [m]

    // find OGE thrust
    if (t > 0 && t <= 30)
    {
        pos_ref(0) = 0;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    if (t > 30 && t < 50)
    {
        pos_ref(0) = 1;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }

    // land
    if (t > 50)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = 1;
        pos_ref(1) = 0;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// oscillations
void oscillations(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    const float offset = 1.25; // vertical offset from IGE to OGE [m]
    const float T_seq = 5;     // time for a certain altitude command [s]
    const float x = 0.5;       // x-coordinate [m]
    const float y = 0.5;       // y-coordinate [m]
    const float h_0 = 0.4;     // heihgt [m]
    const float amp = 0.35;    // amplitude [m]
    const float freq = 0.3;    // frequency [1_s]

    // Taking off
    if (t > 0 && t <= 20)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = 1.5;
        yaw_ref = 0.0;
    }
    // flying in High altitude (OGE)
    if (t > 20 && t < 70)
    {
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = h_0 + amp * std::sin(freq * (t - 20) * (2 * M_PI)) + offset;
        yaw_ref = 0.0;
        int i = (t - 20.0) / T_seq;
    }
    // flying in Low altitude (IGE)
    if (t > 70 && t < 120)
    {
        pos_ref(0) = x;
        pos_ref(1) = y;

        pos_ref(2) = h_0 + amp * std::sin(freq * (t - 70) * (2 * M_PI));
        yaw_ref = 0.0;
        int i = (t - 70.0) / T_seq;
    }

    if (t > 120)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = x;
        pos_ref(1) = y;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

/* DATA COLLECTION TRAJECTORIES */
// quasi static flying for simple GE Model
void staticDataCollection(float t, Eigen::Vector3f &pos,
                          Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // constants
    const float h_0 = 0.9;                    // height before step [m]
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
        pos_ref(0) = x;
        pos_ref(1) = y;
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
        pos_ref(0) = x;
        pos_ref(1) = y;
        yaw_ref = 0.0;
        int i = (t - 70.0) / T_seq;

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

// velocity control
void velocityControl(float t, Eigen::Vector3f &pos, Eigen::Vector3f &pos_ref, float &yaw_ref, Eigen::Vector3f &vel_ref)
{
    const float fwd_speed = 0.3;
    // Taking off
    if (t > 0 && t <= 20)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(0) = -1.5;
        pos_ref(1) = 0;
        pos_ref(2) = 1.5;
        yaw_ref = 0.0;

        // proportional position error
        Eigen::Vector3f pos_p_error = pos_ref - pos;

        //  desired velocity
        vel_ref(0) = 0.95 * pos_p_error(0);
        vel_ref(1) = 0.95 * pos_p_error(1);
        vel_ref(2) = 0.95 * pos_p_error(2); // different gain for Z-error
    }

    // Flying across field
    if (t > 20 && t <= 40)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(1) = 0;
        pos_ref(2) = 1.5;
        yaw_ref = 0.0;

        // proportional position error
        Eigen::Vector3f pos_p_error = pos_ref - pos;

        //  desired velocity
        vel_ref(1) = 0.95 * pos_p_error(1);
        vel_ref(2) = 0.95 * pos_p_error(2); // different gain for Z-error

        if (pos(0) >= 2.1)
        {
            pos_ref(0) = 2;
            vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
        }
        else
        {
            vel_ref(0) = fwd_speed;
        }
    }
    if (t > 40)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = 2.0;
        pos_ref(1) = 0.0;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// velocity control
void velocityStepResponse(float t, Eigen::Vector3f &pos, Eigen::Vector3f &pos_ref, float &yaw_ref, Eigen::Vector3f &vel_ref, float v)
{
    const float h_0 = 1.25;
    const float h_1 = 0.25;
    const float step_location = 0.75;
    // Taking off
    if (t > 0 && t <= 20)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(0) = -1.5;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;

        // proportional position error
        Eigen::Vector3f pos_p_error = pos_ref - pos;

        //  desired velocity
        vel_ref(0) = 0.95 * pos_p_error(0);
        vel_ref(1) = 0.95 * pos_p_error(1);
        vel_ref(2) = 0.95 * pos_p_error(2); // different gain for Z-error
    }

    // Flying across field
    // first step
    if (t > 20 && t <= 30)
    {
        std::cout << "first step" << std::endl;
        pos_ref(1) = 0;
        yaw_ref = 0.0;

        // height
        if (pos(0) >= step_location)
        {
            pos_ref(2) = h_1;
        }
        else
        {
            pos_ref(2) = h_0;
        }

        //  desired velocity
        vel_ref(1) = 0.95 * (pos_ref(1) - pos(1));
        vel_ref(2) = 0.95 * (pos_ref(2) - pos(2)); // different gain for Z-error

        // speed
        if (pos(0) >= 2)
        {
            pos_ref(0) = 2.2;
            vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
        }
        else
        {
            vel_ref(0) = v;
        }
    }
    if (t > 30 && t <= 35)
    {
        pos_ref(0) = -1.5;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // second step
    if (t > 35 && t <= 45)
    {
        std::cout << "second step" << std::endl;
        pos_ref(1) = 0;
        yaw_ref = 0.0;

        // height
        if (pos(0) >= step_location)
        {
            pos_ref(2) = h_1;
        }
        else
        {
            pos_ref(2) = h_0;
        }

        //  desired velocity
        vel_ref(1) = 0.95 * (pos_ref(1) - pos(1));
        vel_ref(2) = 0.95 * (pos_ref(2) - pos(2)); // different gain for Z-error

        // speed
        if (pos(0) >= 2)
        {
            pos_ref(0) = 2.2;
            vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
        }
        else
        {
            vel_ref(0) = v;
        }
    }
    if (t > 45 && t <= 50)
    {
        pos_ref(0) = -1.5;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // third step
    if (t > 50 && t <= 60)
    {
        std::cout << "third step" << std::endl;
        pos_ref(1) = 0;
        yaw_ref = 0.0;

        // height
        if (pos(0) >= step_location)
        {
            pos_ref(2) = h_1;
        }
        else
        {
            pos_ref(2) = h_0;
        }

        //  desired velocity
        vel_ref(1) = 0.95 * (pos_ref(1) - pos(1));
        vel_ref(2) = 0.95 * (pos_ref(2) - pos(2)); // different gain for Z-error

        // speed
        if (pos(0) >= 2)
        {
            pos_ref(0) = 2.2;
            vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
        }
        else
        {
            vel_ref(0) = v;
        }
    }
    if (t > 60 && t <= 65)
    {
        pos_ref(0) = -1.5;
        pos_ref(1) = 0;
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }

    if (t > 65)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = 2.0;
        pos_ref(1) = 0.0;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// velocity data collection
void velocityDataCollection(float t, Eigen::Vector3f &pos, Eigen::Vector3f &pos_ref, float &yaw_ref, Eigen::Vector3f &vel_ref)
{
    const float h[] = {1, 0.5, 0.4, 0.3, 0.2,0.1};
    const float v[] = {0.5, 1.25, 2.0, 2.75};
    const float x_min = -1.5;
    const float x_max = 1.5;
    const float y = 1.0;

    // Taking off
    if (t > 0 && t <= 20)
    {
        std::cout << "taking off" << std::endl;
        pos_ref(0) = x_min;
        pos_ref(1) = y;
        pos_ref(2) = h[0];
        yaw_ref = 0.0;

        // proportional position error
        Eigen::Vector3f pos_p_error = pos_ref - pos;

        //  desired velocity
        vel_ref(0) = 0.95 * pos_p_error(0);
        vel_ref(1) = 0.95 * pos_p_error(1);
        vel_ref(2) = 0.95 * pos_p_error(2); // different gain for Z-error
    }

    // collect Data
    if (t > 20 && t <= 20 + 6 * 60)
    {
        int j = int(t - 20) / 60;          // iterate trought heights
        int i = int(t - 20 - j * 60) / 15; // iterate trought speeds

        std::cout << "j: " << j << "\t height: " << h[j] << "\t i: " << i << "\t speed: " << v[i] << std::endl;
        if (t > 15 * i + 60 * j + 20 && t <= 15 * i + 60 * j + 27.5)
        {
            flyfwd(x_max, h[j], y, v[i], pos, pos_ref, yaw_ref, vel_ref);
        }
        if (t > 15 * i + 60 * j + 27.5 && t <= 15 * i + 60 * j + 35)
        {
            flybwd(x_min, h[j], y, v[i], pos, pos_ref, yaw_ref, vel_ref);
        }
    }

    if (t > 20 + 6 * 60)
    {
        std::cout << "landing now" << std::endl;
        pos_ref(0) = 2.0;
        pos_ref(1) = 0.0;
        pos_ref(2) = 0.0;
        yaw_ref = 0.0;
    }
}

// helper functions
void flyfwd(float x_max, float height, float y, float speed, Eigen::Vector3f &pos, Eigen::Vector3f &pos_ref, float &yaw_ref, Eigen::Vector3f &vel_ref)
{
    pos_ref(1) = y;
    pos_ref(2) = height;
    yaw_ref = 0.0;

    //  desired velocity
    vel_ref(1) = 0.95 * (pos_ref(1) - pos(1));
    vel_ref(2) = 0.95 * (pos_ref(2) - pos(2)); // different gain for Z-error

    // speed
    if (pos(0) >= x_max)
    {
        pos_ref(0) = x_max + 0.6;
        vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
    }
    else
    {
        vel_ref(0) = speed;
    }
}
void flybwd(float x_min, float height, float y, float speed, Eigen::Vector3f &pos, Eigen::Vector3f &pos_ref, float &yaw_ref, Eigen::Vector3f &vel_ref)
{
    pos_ref(1) = y;
    pos_ref(2) = height;
    yaw_ref = 0.0;

    //  desired velocity
    vel_ref(1) = 0.95 * (pos_ref(1) - pos(1));
    vel_ref(2) = 0.95 * (pos_ref(2) - pos(2)); // different gain for Z-error

    // speed
    if (pos(0) <= x_min)
    {
        pos_ref(0) = x_min - 0.6;
        vel_ref(0) = 0.95 * (pos_ref(0) - pos(0));
    }
    else
    {
        vel_ref(0) = -speed;
    }
}
