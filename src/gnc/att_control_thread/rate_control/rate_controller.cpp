#include "gnc/att_control_thread/rate_control/rate_controller.hpp"

namespace gnc {

RateController::RateController(float kp[3], float ki[3], float kd[3], float alpha_d[3],
          float out_max, float out_min,
          float integ_clamp_xy, float integ_clamp_z,
          float dt_ms)
    : pid_x_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[0], ki[0], kd[0], integ_clamp_xy, alpha_d[0]),
      pid_y_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[1], ki[1], kd[1], integ_clamp_xy, alpha_d[1]),
      pid_z_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[2], ki[2], kd[2], integ_clamp_z, alpha_d[2])
{}

Eigen::Vector3f RateController::run(Eigen::Vector3f rate_setpoint, Eigen::Vector3f rate_measurement)
{
    return {
        pid_x_.run(rate_setpoint.x(), rate_measurement.x()),
        pid_y_.run(rate_setpoint.y(), rate_measurement.y()),
        pid_z_.run(rate_setpoint.z(), rate_measurement.z())
    };
}

};