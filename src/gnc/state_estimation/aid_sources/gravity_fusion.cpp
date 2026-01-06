/**
 * @file gravity_fusion.cpp
 * @brief fuse gravity measurement from accelerometer into ESKF if within specific tolerances (non-accelerating)
 */

#include "gnc/state_estimation/eskf.hpp"
#include "gnc/state_estimation/math/helpers.hpp"

namespace gnc {

void ESKF::fuseGravity(const Eigen::Vector3f& accelMeas, const Matrix3f& measCov)
{
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));
    Eigen::Vector3f accel_corrected = (accelMeas - x_.segment<3>(AB_ID));

    // if accel measurement is too small or too large, skip aiding (vehicle is accelerating)
    if (accel_corrected.squaredNorm() < SQ(GRAVITY * 0.9f) || accel_corrected.squaredNorm() > SQ(GRAVITY * 1.1f)) {
        return;
    }

    accel_corrected.normalize();

    // get innovation
    Eigen::Vector3f gravity_pred = current_quat.toRotationMatrix().transpose() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f innov = accel_corrected - gravity_pred;

    // print3DUpdate(accel_corrected, gravity_pred, innov, current_quat);
    Eigen::Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dTHETA_ID) = getSkewSymmetric(gravity_pred);

    // fuse, correct measCov by dividing by gravity squared
    fuseMeasurement3D(innov, measCov / SQ(GRAVITY), H);
}

} // namespace gnc