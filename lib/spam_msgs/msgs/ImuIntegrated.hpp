#ifndef IMU_INTEGRATED_MSG_HPP
#define IMU_INTEGRATED_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param delta_vel delta velocity computed at 250hz for use in the estimator in m/s
 * @param delta_angle delta angle computed at 250hz for use in the estimator in rad
 */
struct ImuIntegratedMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f delta_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f delta_angle = Eigen::Vector3f::Zero();
    float delta_vel_dt = 0.0f; // integration time in seconds
    float delta_angle_dt = 0.0f; // integration time in seconds
};

#endif // IMU_INTEGRATED_MSG_HPP


