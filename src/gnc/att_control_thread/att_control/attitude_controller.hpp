#ifndef ATTITUDE_CONTROLLER_HPP
#define ATTITUDE_CONTROLLER_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"
#include "cpp-pid/pid_dom.hpp"

namespace gnc {

class AttitudeController
{
public:
    /** 
     * @brief Constructor for RateController
     * 
     * @param kp Proportional gains for x, y, z axes
     * @param ki Integral gains for x, y, z axes
     * @param kd Derivative gains for x, y, z axes
     * @param alpha_d Derivative smoothing factors for x, y, z axes
     * @param out_max Maximum output value
     * @param out_min Minimum output value
     * @param integ_clamp_xy Integral clamp value for x and y axes
     * @param integ_clamp_z Integral clamp value for z axis
     * @param dt_ms Time step in milliseconds
     */
    AttitudeController(float kp[3], float ki[3], float kd[3], float alpha_d[3],
                   float out_max, float out_min,
                   float integ_clamp_xy, float integ_clamp_z,
                   float dt_ms);
    ~AttitudeController() = default;
    
    Eigen::Vector3f run(Eigen::Quaternionf attitude_setpoint, Eigen::Quaternionf attitude_measurement);
    void reset();

private:
    PID_DOM pid_x_;
    PID_DOM pid_y_;
    PID_DOM pid_z_;
};

} // namespace gnc

#endif // ATTITUDE_CONTROLLER_HPP