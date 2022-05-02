#include "trajectories.h"
/* BENCHMARK TRAJECTORIES */
// vertical step response from 1.20 m to 0.20 m
void stepResponse(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref)
{
    if (t > 0 && t <= 20)
    { // takeoff
        pos_ref(0) = 0;
        pos_ref(1) = 0;
        pos_ref(2) = 1.20;
        yaw_ref = 0.0;
    }
    // step response
    if (t > 25 && t <= 55)
    {
        pos_ref(0) = 0;
        pos_ref(1) = 0;
        pos_ref(2) = 0.20;
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
    // TODO
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
