#include "gnc/state_estimation/eskf.hpp"
#include "gnc/state_estimation/math/helpers.hpp"

namespace gnc {

void ESKF::fuseMag(const Eigen::Vector3f& magMeas, const float& R)
{
    // predicted body mag measurement (measurement model)
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));

    Eigen::Vector3f mag_meas_nav = current_quat.toRotationMatrix() * magMeas; // rotate measured mag to nav frame // current quat takes body to nav
    
    // this should be exactly north, if its off, then we have an innovation
    float meas_yaw = atan2f(-mag_meas_nav.y(), mag_meas_nav.x()); // yaw measurement from north
    
    // innovation
    float yaw_innov =  meas_yaw;
    // wrap to [-pi, pi]
    if (yaw_innov > M_PI) {
        yaw_innov -= 2.0f * M_PI;
    } else if (yaw_innov < -M_PI) {
        yaw_innov += 2.0f * M_PI;
    }

    Eigen::Vector3f innov = Eigen::Vector3f(0.0f, 0.0f, yaw_innov); // only yaw innovation, no roll/pitch info from mag

    // print3DUpdate(mag_meas_nav, Eigen::Vector3f(meas_yaw, 0.0f, 0.0f), innov, current_quat);

    // Jacob. of measurement model -> H = H_x X_dx, where X_dx is the jacobian of state to error state
    Eigen::Matrix<float, 3, dSTATE_SIZE> H = Eigen::Matrix<float, 3, dSTATE_SIZE>::Zero();
    H(2, dTHETA_ID+2) = 1.0f; // only yaw is measured

    // Eigen::Matrix<float, 3, 4> H_x;
    // Eigen::Vector3f dh_dq_w = 2*(current_quat.w()*init_mag_nav_ + current_quat.vec().cross(init_mag_nav_));
    // Eigen::Matrix3f dh_dq_v = 2*(c`urrent_quat.vec().transpose()*init_mag_nav_*Eigen::Matrix3f::Identity()
    //                             + current_quat.vec()*init_mag_nav_.transpose()
    //                             - init_mag_nav_*current_quat.vec().transpose()
    //                             - current_quat.w()*getSkewSymmetric(init_mag_nav_));
    // H_x.col(0) = dh_dq_v.col(0);
    // H_x.col(1) = dh_dq_v.col(1);
    // H_x.col(2) = dh_dq_v.col(2);
    // H_x.col(3) = dh_dq_w;

    // Eigen::Matrix<float, 4, 3> dq_dtheta;
    // float qx = current_quat.x();
    // float qy = current_quat.y();
    // float qz = current_quat.z();
    // float qw = current_quat.w();

    // dq_dtheta << -qx, -qy, -qz,
    //             qw, -qz,  qy,
    //             qz,  qw, -qx,
    //             -qy,  qx,  qw;

    // dq_dtheta *= 0.5f;

#ifdef DEBUG_MAG_FUSION
    print3DUpdate(magMeas, mag_pred, innov, current_quat);
#endif

    Eigen::Vector3f S = fuseAttitude3D(innov, R, H, 1.0f); // 1.0f innov gate (very bad noise in myhal arena (full of rebar!!))

    ekf_innovations_msg.timestamp = micros();
    ekf_innovations_msg.mag_innov = innov;
    ekf_innovations_msg.mag_innov_cov = S;
}

} // namespace gnc