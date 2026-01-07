#ifndef FORCE_SETPOINT_MSG_HPP
#define FORCE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Motor forces setpoint in Newtons
 */
struct ForceSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f motor1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f motor2 = Eigen::Vector3f::Zero();
};

#endif // FORCE_SETPOINT_MSG_HPP