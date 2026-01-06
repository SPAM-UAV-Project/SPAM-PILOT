#ifndef RATE_SETPOINT_MSG_HPP
#define RATE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param rate_setpoint Rate setpoint in radians per second (x,y,z)
 */
struct RateSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f rate_setpoint = Eigen::Vector3f::Zero();    
};

#endif // RATE_SETPOINT_MSG_HPP