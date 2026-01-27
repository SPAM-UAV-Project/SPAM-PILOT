#ifndef TORQUE_SETPOINT_MSG_HPP
#define TORQUE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Body torque setpoint in Nm
 */
struct TorqueSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f setpoint = Eigen::Vector3f::Zero();
};

#endif // TORQUE_SETPOINT_MSG_HPP