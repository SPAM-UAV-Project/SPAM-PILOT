#ifndef TORQUE_SETPOINT_MSG_HPP
#define TORQUE_SETPOINT_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint Body torque setpoint in Nm
 * @param delayed_torque Delayed torque estimate (actuator model + filter) in Nm
 * @param indi_increment INDI delta_u increment in Nm
 */
struct TorqueSetpointMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f setpoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f delayed_torque = Eigen::Vector3f::Zero();
    Eigen::Vector3f indi_increment = Eigen::Vector3f::Zero();
};

#endif // TORQUE_SETPOINT_MSG_HPP