#ifndef RATE_CONTROLLER_HPP
#define RATE_CONTROLLER_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"
#include "cpp-pid/pid_dom.hpp"

namespace gnc {

class RateController
{
public:
    RateController();
    ~RateController() = default;

    void init();

private:
    void rateControlTask(void *pvParameters);
    static void rateControlTaskEntry(void *instance) {
        static_cast<RateController *>(instance)->rateControlTask(instance);
    }

    Eigen::Vector3f run(Eigen::Vector3f rate_setpoint, Eigen::Vector3f rate_measurement);

    const uint32_t dt_ = 2; // ms -> 500 Hz

    PID_DOM pid_x_;
    PID_DOM pid_y_;
    PID_DOM pid_z_;
    Eigen::Vector3f control_output_;

    float kp_[3] = {0.1f, 0.1f, 0.1f};
    float ki_[3] = {0.f, 0.f, 0.f};
    float kd_[3] = {0.f, 0.f, 0.f};
    float alpha_d_[3] = {0.9f, 0.9f, 0.9f}; // derivative smoothing factor

    float out_max_ = 1.0f, out_min_ = -1.0f;
    float integ_clamp_xy_ = 0.2f;
    float integ_clamp_z_ = 0.5f;



};

} // namespace gnc

#endif // RATE_CONTROLLER_HPP