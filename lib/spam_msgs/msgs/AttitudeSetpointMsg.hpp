#ifndef ATTITUDE_SETPOINT_MSG_HPP
#define ATTITUDE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Desired attitude setpoint as a quaternion
 */
struct AttitudeSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Quaternionf q_sp = Eigen::Quaternionf::Identity();
    float yaw_sp_ff_rate = 0.0f; // normalized yaw rate feedforward for RC yaw stick
};

#endif // ATTITUDE_SETPOINT_MSG_HPP