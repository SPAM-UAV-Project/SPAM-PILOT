#ifndef RATE_SETPOINT_MSG_HPP
#define RATE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Desired attitude setpoint as a vector of roll, pitch and yaw in radians per second
 */
struct RateSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f setpoint = Eigen::Vector3f::Zero(); // rpy, radians per second
};

#endif // RATE_SETPOINT_MSG_HPP