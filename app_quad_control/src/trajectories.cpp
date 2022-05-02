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
    if (t > 25 && t <= 55)
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
    const float speed = 1.0;                              // horizontal speed [m_s]
    const float circle_Radius = 1.0;                      // radius of flown circle [m]
    const float angular_velocity = speed / circle_Radius; // angular velocity [1_s]
    const float circle_center_x = 1.0;                    // center of circle [m]
    const float circle_center_y = 1.0;                    // center of circle [m]

    // takeoff and go to start
    if (t > 0 && t <= 20)
    {
        pos_ref(0) = std::cos(0);
        pos_ref(1) = std::sin(0);
        pos_ref(2) = h_0;
        yaw_ref = 0.0;
    }
    // start flying circles
    if (t > 20 && t <= 40)
    {
        pos_ref(0) = std::cos(angular_velocity * t);
        pos_ref(1) = std::sin(angular_velocity * t);
        pos_ref(2) = h_0;
        yaw_ref = angular_velocity * t;
    }
    // step down while flying circles
    if (t > 40 && t <= 60)
    {
        pos_ref(0) = std::cos(angular_velocity * t);
        pos_ref(1) = std::sin(angular_velocity * t);
        pos_ref(2) = h_1;
        yaw_ref = angular_velocity * t;
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
    // TODO
}

// vertical speed flying for more advanced GE Model
void verticalSpeedDataCollection(float t, Eigen::Vector3f &pos,
                                 Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    const float offset = 1.25; // vertical offset from IGE to OGE [m]
    // get out of ground effect thrust (OGE)
    // TODO
    // get out in ground effect thrust (IGE)
    // TODO
}

// omnidirectional flying for most advanced GE Model
void allSpeedDataCollection(float t, Eigen::Vector3f &pos,
                            Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    // TODO
}
