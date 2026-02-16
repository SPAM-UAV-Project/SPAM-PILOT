#ifndef ANG_ACCEL_MSG_HPP
#define ANG_ACCEL_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param ang_accel_raw Raw angular acceleration in rad/s^2
 * @param ang_accel_filtered Filtered angular acceleration in rad/s^2
 */
struct AngAccelMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f ang_accel_raw = Eigen::Vector3f::Zero();
    Eigen::Vector3f ang_accel_filtered = Eigen::Vector3f::Zero();
};

#endif // ANG_ACCEL_MSG_HPP
