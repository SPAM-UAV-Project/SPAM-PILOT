#ifndef IMU_MAG_MSG_HPP
#define IMU_MAG_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param mag Magnetometer readings in microteslas 
 */
struct ImuMagMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f mag = Eigen::Vector3f::Zero();
};

#endif // IMU_MAG_MSG_HPP

