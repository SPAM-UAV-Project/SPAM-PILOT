#include "gnc/vel_control_thread/vel_control/vel_controller.hpp"

namespace gnc {

VelController::VelController(float kp[3], float ki[3], float kd[3], float alpha_d[3],
          float out_max, float out_min,
          float integ_clamp_xy, float integ_clamp_z,
          float dt_ms)
    : pid_x_(dt_ms * 0.001f, out_max, out_min, 1.0f, kp[0], ki[0], kd[0], integ_clamp_xy, alpha_d[0]),
      pid_y_(dt_ms * 0.001f, out_max, out_min, 1.0f, kp[1], ki[1], kd[1], integ_clamp_xy, alpha_d[1]),
      pid_z_(dt_ms * 0.001f, out_max, out_min, 1.0f, kp[2], ki[2], kd[2], integ_clamp_z, alpha_d[2])
{}

Eigen::Vector3f VelController::run(Eigen::Vector3f vel_setpoint, Eigen::Vector3f vel_measurement, Eigen::Quaternionf cur_attitude)
{
    // transform EKF nav velocity into body frame
    Eigen::Vector3f vel_meas_body = cur_attitude.toRotationMatrix().transpose() * vel_measurement;

    return {
        pid_x_.run(vel_setpoint.x(), vel_meas_body.x()),
        pid_y_.run(vel_setpoint.y(), vel_meas_body.y()),
        pid_z_.run(vel_setpoint.z(), vel_meas_body.z())
    };
}

void VelController::reset() 
{
    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset(); 
}

};