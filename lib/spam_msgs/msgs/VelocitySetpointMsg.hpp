#ifndef VELOCITY_SETPOINT_MSG_HPP
#define VELOCITY_SETPOINT_MSG_HPP

#include "msg_broker.hpp"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Desired velocity setpoint
 */
struct VelocitySetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f setpoint = Eigen::Vector3f::Zero();
    float yaw_sp_ff_rate = 0.0f; // normalized yaw rate feedforward for RC yaw stick
};

#endif // VELOCITY_SETPOINT_MSG_HPP