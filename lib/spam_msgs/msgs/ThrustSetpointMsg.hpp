#ifndef THRUST_SETPOINT_MSG_HPP
#define THRUST_SETPOINT_MSG_HPP

#include "msg_broker.hpp"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Normalized thrust between 0 and 1
 */
struct ThrustSetpointMsg
{
    uint64_t timestamp = 0;
    float setpoint = 0.0f;
};

#endif // THRUST_SETPOINT_MSG_HPP