/**
 * @file gravity_fusion.cpp
 * @brief fuse gravity measurement from accelerometer into ESKF if within specific tolerances (non-accelerating)
 */

#include "gnc/state_estimation/eskf.hpp"
#include "gnc/state_estimation/math/helpers.hpp"
#include "msgs/ImuIntegrated.hpp"

namespace {
    constexpr float _gbound_l = SQ(GRAVITY * 0.9f);
    constexpr float _gbound_h = SQ(GRAVITY * 1.1f);
    constexpr float _g_invsq = 1.0f / SQ(GRAVITY);
}

namespace gnc {

void ESKF::fuseGravity(const Vector3f& accel_meas, const Eigen::Vector3f& accel_meas_filtered_, const float& R){
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));
    // normalize measurement to unit vector since we only care about direction
    Eigen::Vector3f accel_corrected = ((accel_meas - x_.segment<3>(AB_ID)).normalized());

    // if filtered accel measurement is too small or too large, skip aiding (vehicle is accelerating)
    // filter here since accel data is probably way too noisy for this application
    // Serial.println("Gravity fusion: accel norm = " + String(accel_meas_filtered_.norm()));
    if (accel_meas_filtered_.squaredNorm() < _gbound_l || accel_meas_filtered_.squaredNorm() > _gbound_h) {
        return;
    }

    // get innovation
    Eigen::Vector3f gravity_pred = current_quat.toRotationMatrix().transpose() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f innov = accel_corrected - gravity_pred;

    // print3DUpdate(accel_corrected, gravity_pred, innov, current_quat);
    Eigen::Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dTHETA_ID) = getSkewSymmetric(gravity_pred);

    // fuse, correct measCov by dividing by gravity squared
    Eigen::Vector3f S = fuseAttitude3D(innov, R * _g_invsq, H);

    ekf_innovations_msg.timestamp = micros();
    ekf_innovations_msg.gravity_innov = innov;
    ekf_innovations_msg.gravity_innov_cov = S;
}

} // namespace gnc