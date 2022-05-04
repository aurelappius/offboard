#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
/* BENCHMARK TRAJECTORIES */
// vertical step response from 1.20 m to 0.20 m
void stepResponse(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref);

// vertical step response from 1.20 m to 0.20 m with forward speed
void circleStepResponse(float t, Eigen::Vector3f &pos,
                        Eigen::Vector3f &pos_ref, float &yaw_ref);

// oscillations
void oscillations(float t, Eigen::Vector3f &pos,
                  Eigen::Vector3f &pos_ref, float &yaw_ref);

/* DATA COLLECTION TRAJECTORIES */
// quasi static flying for simple GE Model
void staticDataCollection(float t, Eigen::Vector3f &pos,
                          Eigen::Vector3f &pos_ref, float &yaw_ref);

// vertical speed flying for more advanced GE Model
void verticalSpeedDataCollection(float t, Eigen::Vector3f &pos,
                                 Eigen::Vector3f &pos_ref, float &yaw_ref);

// omnidirectional flying for most advanced GE Model
void allSpeedDataCollection(float t, Eigen::Vector3f &pos,
                            Eigen::Vector3f &pos_ref, float &yaw_ref);