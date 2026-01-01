/**
 * @file eskf.hpp
 * @brief class for the eskf attitude & position estimator. see: https://arxiv.org/pdf/1711.02508 
 */

#ifndef ESKF_HPP
#define ESKF_HPP

#include <ArduinoEigen/Eigen/Dense>

using namespace Eigen;

#define POS_ID (0)
#define VEL_ID (POS_ID + 3)
#define QUAT_ID (VEL_ID + 3)
#define AB_ID (QUAT_ID + 4)
#define GB_ID (AB_ID + 3)
#define STATE_SIZE (GB_ID + 3)

#define dPOS_ID (0)
#define dVEL_ID (dPOS_ID + 3)
#define dTHETA_ID (dVEL_ID + 3)
#define dAB_ID (dTHETA_ID + 3)
#define dGB_ID (dAB_ID + 3)
#define dSTATE_SIZE (dGB_ID + 3)

#define SQ(x) ((x)*(x))
#define I_3 Eigen::Matrix3f::Identity()
#define I_STATE Eigen::Matrix<float, STATE_SIZE, STATE_SIZE>::Identity()

#define GRAVITY (9.81f)

namespace gnc {

class ESKF
{
public:
    ESKF();
    ~ESKF() = default;
    
    void initStates(const Vector3f& init_pos = Vector3f::Zero(),
                    const Vector3f& init_vel = Vector3f::Zero(),
                    const Vector4f& init_quat = (Vector4f() << 0.0f, 0.0f, 0.0f, 1.0f).finished(),
                    const Vector3f& init_ab = Vector3f::Zero(),
                    const Vector3f& init_gb = Vector3f::Zero());
    void initCovariances(const Matrix3f& pos_cov, const Matrix3f& vel_cov,
                         const Matrix3f& attitude_cov, const Matrix3f& ab_cov,
                         const Matrix3f& gb_cov);
    void initSensorNoise(const float accel_noise_var, const float accel_walk_var,
                         const float gyro_noise_var, const float gyro_walk_var);
    void predictStates(const Vector3f& accelMeas, const Vector3f& gyroMeas, float dt);

    Eigen::VectorXf getStateVariable(int id, int length) {
        return x_.segment(id, length);
    }

private:
    Eigen::Matrix<float, STATE_SIZE, 1> x_; // nominal state
    Eigen::Matrix<float, dSTATE_SIZE, 1> dx_; // error state

    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P_; // covariance of the error state
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> F_; // error state transition matrix

    // process noise covariance in place of Q
    float accel_noise_var_, accel_walk_var_;
    float gyro_noise_var_, gyro_walk_var_;

    // mag bias
    Eigen::Vector3f mag_bias_ = Eigen::Vector3f::Zero();
};

}

#endif // ESKF_HPP
 
