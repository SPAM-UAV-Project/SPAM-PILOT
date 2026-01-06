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
    Eigen::Quaternionf setpoint = Eigen::Quaternionf::Identity();   
};

#endif // ATTITUDE_SETPOINT_MSG_HPP