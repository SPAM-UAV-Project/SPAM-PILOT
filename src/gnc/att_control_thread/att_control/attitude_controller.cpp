#include "gnc/att_control_thread/att_control/attitude_controller.hpp"

namespace gnc {

    AttitudeController::AttitudeController(float kp[3], float ki[3], float kd[3], float alpha_d[3],
               float out_max, float out_min,
               float integ_clamp_xy, float integ_clamp_z,
               float dt_ms)
        : pid_x_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[0], ki[0], kd[0], integ_clamp_xy, alpha_d[0]),
          pid_y_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[1], ki[1], kd[1], integ_clamp_xy, alpha_d[1]),
          pid_z_(dt_ms / 1000.0f, out_max, out_min, 1.0f, kp[2], ki[2], kd[2], integ_clamp_z, alpha_d[2])
    {}

    Eigen::Vector3f AttitudeController::run(Eigen::Quaternionf attitude_setpoint, Eigen::Quaternionf attitude_measurement)
    {
        // calculate the error quaternion, send the x,y,z components as setpoints with 0 measurement
        error_quat_ = (attitude_setpoint * attitude_measurement.conjugate()).normalized();

        if (error_quat_.w() < 0.0f) {
            error_quat_.coeffs() *= -1.0f;
        }

        return {
            // run pid with 2 * the vector axis for a small angle approximation
            pid_x_.run(2 * error_quat_.x(), 0.0f),
            pid_y_.run(2 * error_quat_.y(), 0.0f),
            pid_z_.run(2 * error_quat_.z(), 0.0f)
        };
    }

    void AttitudeController::reset() 
    {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset(); 
    }
}