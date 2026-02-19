#ifndef MOTOR_FORCES_MSG_HPP
#define MOTOR_FORCES_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param force_setpoint 4x1 vector of motor forces in Newtons
 * @param actuator_setpoints 4x1 vector of actuator commands (blade_x, blade_y, u_top, u_bot)
 */
struct MotorForcesMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector4f force_setpoint = Eigen::Vector4f::Zero();
    Eigen::Vector4f actuator_setpoints = Eigen::Vector4f::Zero(); 
};

#endif // MOTOR_FORCES_MSG_HPP