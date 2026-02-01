#ifndef IMU_HIGH_RATE_MSG_HPP
#define IMU_HIGH_RATE_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param accel Accelerometer readings in m/s^2
 * @param gyro Gyroscope readings in rad/s 
 */
struct ImuHighRateMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f accel = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_filtered = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_filtered = Eigen::Vector3f::Zero();
};

#endif // IMU_HIGH_RATE_MSG_HPP


