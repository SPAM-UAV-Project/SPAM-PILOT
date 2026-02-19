#ifndef EKF_STATES_MSG_HPP
#define EKF_STATES_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param position Position in meters (NED)
 * @param velocity Velocity in meters per second (NED)
 * @param attitude Quaternion representing orientation (x,y,z,w)
 * @param accel_bias Accelerometer bias in m/s^2
 * @param gyro_bias Gyroscope bias in rad/s
 */
struct EkfStatesMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Quaternionf attitude = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);    
    Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
};

#endif // EKF_STATES_MSG_HPP


