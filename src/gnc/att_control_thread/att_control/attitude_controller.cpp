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
        Eigen::Quaternionf error_quat = (attitude_measurement.conjugate() * attitude_setpoint).normalized();

        // ensure shortest path
        if (error_quat.w() < 0.0f) { 
            error_quat.coeffs() *= -1.0f;
        }

        // separate the tilt component and the twist component to ensure tilt control is maintained
        float twist_norm_sq = error_quat.w() * error_quat.w() + error_quat.z() * error_quat.z();
        Eigen::Quaternionf twist_quat;
        if (twist_norm_sq > 1e-6f){
            twist_quat = Eigen::Quaternionf(error_quat.w(), 0.0f, 0.0f, error_quat.z()).normalized();
        } else {
            twist_quat = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
        }
        Eigen::Quaternionf tilt_quat = error_quat * twist_quat.conjugate();

        return {
            // run pid with 2 * the vector axis for a small angle approximation
            pid_x_.run(2 * tilt_quat.x(), 0.0f),
            pid_y_.run(2 * tilt_quat.y(), 0.0f),
            pid_z_.run(2 * twist_quat.z(), 0.0f)
        };
    }

    void AttitudeController::reset() 
    {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset(); 
    }
}