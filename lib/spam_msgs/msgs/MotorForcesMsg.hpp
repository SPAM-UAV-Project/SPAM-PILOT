#ifndef MOTOR_FORCES_MSG_HPP
#define MOTOR_FORCES_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param setpoint 4x1 vector of motor forces in Newtons
 */
struct MotorForcesMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector4f setpoint = Eigen::Vector4f::Zero();
};

#endif // MOTOR_FORCES_MSG_HPP