/**
 * @file eskf.cpp
 * @brief Implementation of the eskf attitude & position estimator class
 */

#include "gnc/state_estimation/ESKF.hpp"
#include "gnc/state_estimation/math/helpers.hpp"
#include <Arduino.h>

namespace gnc {

ESKF::ESKF() {}

void ESKF::initStates(const Vector3f& init_pos,
                      const Vector3f& init_vel,
                      const Vector4f& init_quat,
                      const Vector3f& init_ab,
                      const Vector3f& init_gb,
                      const Vector3f& init_mag)
{
    x_.segment<3>(POS_ID) = init_pos; // position
    x_.segment<3>(VEL_ID) = init_vel; // velocity
    x_.segment<4>(QUAT_ID) = init_quat; // quaternion
    x_.segment<3>(AB_ID) = init_ab; // accelerometer bias
    x_.segment<3>(GB_ID) = init_gb; // gyroscope bias
    init_mag_nav_ = init_mag; // initial mag measurement in nav frame
}

void ESKF::initCovariances(const Matrix3f& pos_cov, const Matrix3f& vel_cov,
                         const Matrix3f& attitude_cov, const Matrix3f& ab_cov,
                         const Matrix3f& gb_cov) 
{
    P_.setZero();
    P_.block<3,3>(dPOS_ID, dPOS_ID) = pos_cov; // position
    P_.block<3,3>(dVEL_ID, dVEL_ID) = vel_cov; // velocity
    P_.block<3,3>(dTHETA_ID, dTHETA_ID) = attitude_cov; // attitude
    P_.block<3,3>(dAB_ID, dAB_ID) = ab_cov; // accelerometer bias
    P_.block<3,3>(dGB_ID, dGB_ID) = gb_cov; // gyroscope bias
}
    
void ESKF::initSensorNoise(const float accel_noise_var, const float accel_walk_var,
                         const float gyro_noise_var, const float gyro_walk_var) 
{
    accel_noise_var_ = accel_noise_var;
    accel_walk_var_ = accel_walk_var;
    gyro_noise_var_ = gyro_noise_var;
    gyro_walk_var_ = gyro_walk_var;
}

void ESKF::predictStates(const Vector3f& accelMeas, const Vector3f& gyroMeas, float dt)
{
    Eigen::Vector3f gyro_corrected = gyroMeas - x_.segment<3>(GB_ID);
    Eigen::Vector3f accel_corrected = accelMeas - x_.segment<3>(AB_ID);

    // predict orientations
    Eigen::Vector3f delta_angle = (gyro_corrected) * dt;

    Eigen::Quaternionf delta_quat;
    float angle_mag = delta_angle.norm();
    if (angle_mag > 1e-9) {
        // create a quaternion from the delta angle
        delta_quat = Eigen::Quaternionf(Eigen::AngleAxisf(angle_mag, delta_angle.normalized()));
    } else {
        delta_quat.setIdentity();
    }
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));
    Eigen::Matrix3f R_delta_angle = delta_quat.toRotationMatrix();
    x_.segment<4>(QUAT_ID) = Eigen::Vector4f((current_quat * delta_quat).normalized().coeffs());

    // convert acceleration into nav frame, remove gravity vector
    Eigen::Matrix3f C_n_b = current_quat.toRotationMatrix();
    Eigen::Vector3f accel_nav = C_n_b * (accel_corrected);
    accel_nav.z() += GRAVITY; // remove gravity vector (reads -g on flat surface)

    // predict position and velocity
    x_.segment<3>(POS_ID) += x_.segment<3>(VEL_ID) * dt + 0.5f * accel_nav * dt * dt;
    x_.segment<3>(VEL_ID) += accel_nav * dt;

    // State transition matrix formulated with reference to this paper: https://arxiv.org/pdf/1711.02508
    F_.block<3, 3>(dPOS_ID, dVEL_ID) = Eigen::Matrix3f::Identity() * dt; // dp/dv
    F_.block<3, 3>(dVEL_ID, dTHETA_ID) = -C_n_b * getSkewSymmetric(accel_corrected) * dt;   // dv/dθ
    F_.block<3, 3>(dVEL_ID, dAB_ID) = -C_n_b * dt;             // dv/da_b
    F_.block<3, 3>(dTHETA_ID, dTHETA_ID) =  R_delta_angle.transpose(); // dθ/dθ
    F_.block<3, 3>(dTHETA_ID, dGB_ID) = -Eigen::Matrix3f::Identity() * dt; // dθ/dw_b
    P_ = F_ * P_ * F_.transpose();

    // inject noise (Q matrix)
    P_.diagonal().block<3, 1>(dVEL_ID, 0).array() += accel_noise_var_ * SQ(dt);
    P_.diagonal().block<3, 1>(dTHETA_ID, 0).array() += gyro_noise_var_ * SQ(dt);
    P_.diagonal().block<3, 1>(dAB_ID, 0).array() += accel_walk_var_ * dt;
    P_.diagonal().block<3, 1>(dGB_ID, 0).array() += gyro_walk_var_ * dt;
}

void ESKF::fuseMeasurement3D(const Vector3f& innov,
    const Matrix3f& measCov,
    const Matrix<float, 3, dSTATE_SIZE>& H)
{
    // Kalman gain
    Eigen::Matrix<float, dSTATE_SIZE, 3> PHt = P_*H.transpose();
    Eigen::Matrix<float, dSTATE_SIZE, 3> K = PHt * (H*PHt + measCov).inverse();
    // Correction error state
    Eigen::Matrix<float, dSTATE_SIZE, 1> errorState = K * innov;
    // Update P (simple form)
    // P = (I_dx - K*H)*P;
    // Update P (Joseph form)
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dSTATE - K*H;
    P_ = I_KH*P_*I_KH.transpose() + K*measCov*K.transpose();

    injectCorrection(errorState);
}

void ESKF::injectCorrection(const Eigen::Matrix<float, dSTATE_SIZE, 1>& dx) 
{
    // Inject error state into nominal state (eqn 282, pg 62)
    x_.segment<3>(POS_ID) += dx.block<3, 1>(dPOS_ID, 0);
    x_.segment<3>(VEL_ID) += dx.block<3, 1>(dVEL_ID, 0);
    Eigen::Vector3f dtheta = dx.block<3, 1>(dTHETA_ID, 0);
    Eigen::Quaternionf q_dtheta = Eigen::Quaternionf(Eigen::AngleAxisf(dtheta.norm(), dtheta.normalized()));
    Eigen::Quaternionf current_quat(x_.segment<4>(QUAT_ID));
    x_.segment<4>(QUAT_ID) = (current_quat * q_dtheta).normalized().coeffs();
    x_.segment<3>(AB_ID) += dx.block<3, 1>(dAB_ID, 0);
    x_.segment<3>(GB_ID) += dx.block<3, 1>(dGB_ID, 0);

    // reset error state

}

} // namespace gnc

