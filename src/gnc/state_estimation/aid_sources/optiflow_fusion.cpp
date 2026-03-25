/**
 * @file optiflow_fusion.cpp
 * @brief fuse optical flow velocity measurements
 */

#include "gnc/state_estimation/eskf.hpp"
#include "gnc/state_estimation/math/helpers.hpp"

namespace gnc {

void ESKF::fuseOptiflow(const Eigen::Vector3f& vel_meas, const float& R) {
    // get current attitude to rotate measurement to body frame
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));

    // measurement model - C_b_n * v_nav to get vel into body frame
    Eigen::Vector3f vel_pred = current_quat.toRotationMatrix().transpose() * x_.segment<3>(VEL_ID);
    Eigen::Vector3f innov = vel_meas - vel_pred;

    // print3DUpdate(vel_meas, vel_pred, innov, current_quat);
    Eigen::Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    // TODO: compute jacobian for observation of velocity and attitude coupling

    // fuse
    Eigen::Vector3f S = fuse3D(innov, R, H, 10.0f); // no innov gate for now

    ekf_innovations_msg.timestamp = micros();
    ekf_innovations_msg.optiflow_innov = innov;
    ekf_innovations_msg.optiflow_innov_cov = S;


}

} // namespace gnc