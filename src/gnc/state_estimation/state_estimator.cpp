/**
 * @file state_estimator.cpp
 * @brief Implementation of the state estimation thread for estimating vehicle position and attitude
 */

#include "gnc/state_estimation/state_estimator.hpp"
#include "gnc/state_estimation/ESKF.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ImuMagMsg.hpp"

namespace gnc {

StateEstimator::StateEstimator() {}

void StateEstimator::getInitialStates()
{
    Eigen::Vector3f accel_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f mag_sum = Eigen::Vector3f::Zero();

    float imu_hr_samples = 0.f; 
    float imu_mag_samples = 0.f;

    while (imu_hr_samples < 250 || imu_mag_samples < 100) {
        if (imu_gyro_accel_sub_.pull_if_new(imu_gyro_accel_msg_)) {
            gyro_sum.x() += imu_gyro_accel_msg_.gyro.x();
            gyro_sum.y() += imu_gyro_accel_msg_.gyro.y();
            gyro_sum.z() += imu_gyro_accel_msg_.gyro.z();

            accel_sum.x() += imu_gyro_accel_msg_.accel.x();
            accel_sum.y() += imu_gyro_accel_msg_.accel.y();
            accel_sum.z() += imu_gyro_accel_msg_.accel.z();
            imu_hr_samples++;
        }
        if (imu_mag_sub_.pull_if_new(imu_mag_msg_)) {
            // can use mag data later for TRIAD
            mag_sum.x() += imu_mag_msg_.mag.x();
            mag_sum.y() += imu_mag_msg_.mag.y();
            mag_sum.z() += imu_mag_msg_.mag.z();
            imu_mag_samples++;
        }
    }
    Eigen::Vector3f avg_gyro = gyro_sum / imu_hr_samples;
    Eigen::Vector3f avg_accel = accel_sum / imu_hr_samples;
    Eigen::Vector3f avg_mag = mag_sum / imu_mag_samples;

    // print all initial values
    Serial.printf("[StateEstimator] Initial Gyro Bias: [%.4f, %.4f, %.4f] rad/s\n",
                  avg_gyro.x(), avg_gyro.y(), avg_gyro.z());
    Serial.printf("[StateEstimator] Initial Accel (incl. gravity): [%.4f, %.4f, %.4f] m/s^2\n",
                    avg_accel.x(), avg_accel.y(), avg_accel.z());
    Serial.printf("[StateEstimator] Initial Mag: [%.4f, %.4f, %.4f] uT\n",
                    avg_mag.x(), avg_mag.y(), avg_mag.z());


    // compute initial attitude using accel and mag
    Vector3f mag_unit = avg_mag.normalized();
    Vector3f accel_unit = avg_accel.normalized();

    Vector3f Z = -accel_unit;
    Vector3f Y = Z.cross(mag_unit).normalized();
    Vector3f X = Y.cross(Z).normalized();

    Eigen::Matrix3f C_b_n; // rotation matrix from nav frame to body frame
    C_b_n.col(0) = X;
    C_b_n.col(1) = Y;
    C_b_n.col(2) = Z;

    init_quat_ = Eigen::Quaternionf(C_b_n.transpose()).normalized().coeffs(); // x, y, z, w
    init_gyro_bias_ = avg_gyro;
    init_mag_nav_ = C_b_n.transpose() * avg_mag;

    Serial.printf("[StateEstimator] Initial Attitude Quaternion: [%.4f, %.4f, %.4f, %.4f]\n",
                  init_quat_.x(), init_quat_.y(), init_quat_.z(), init_quat_.w());
    Serial.printf("[StateEstimator] Initial Mag in Nav Frame: [%.4f, %.4f, %.4f] uT\n",
                  init_mag_nav_.x(), init_mag_nav_.y(), init_mag_nav_.z());
    
}

void StateEstimator::init() 
{
    getInitialStates();

    // initialize eskf class 
    eskf_.initStates(Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(),
                     init_quat_,
                     Eigen::Vector3f::Zero(),
                     init_gyro_bias_,
                     init_mag_nav_);
    eskf_.initCovariances(I_3 * pos_var_init_, I_3 * vel_var_init_,
                          I_3 * attitude_var_init_, I_3 * ab_var_init_,
                          I_3 * gb_var_init_);
    eskf_.initSensorNoise(accel_noise_var_, accel_walk_var_,
                          gyro_noise_var_, gyro_walk_var_);

    // start the estimator
    xTaskCreate(
        StateEstimator::stateEstimatorTaskEntry,
        "StateEstimatorTask",
        16384,
        this,
        3,
        &stateEstimatorTaskHandle_
    );

    Serial.println("[StateEstimator] State Estimator Task Started.");
}

void StateEstimator::stateEstimatorTask(void *pvParameters) 
{
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    long last_loop_time = micros();
    long current_time = micros();

    while (true) {
        // obtain sensor data
        if (imu_gyro_accel_sub_.pull_if_new(imu_gyro_accel_msg_)) {
            // compute dt
            current_time = micros();
            dt_ = (current_time - last_loop_time) * 1e-6f; // convert to seconds
            last_loop_time = current_time;

            // obtain prior estimate
            eskf_.predictStates(imu_gyro_accel_msg_.accel,
                                    imu_gyro_accel_msg_.gyro,
                                    dt_);

            // fuse gravity 
            eskf_.fuseGravity(imu_gyro_accel_msg_.accel, Eigen::Matrix3f(accel_noise_var_ * I_3));
        }

        // if recieved mag data, update the filter
        if (imu_mag_sub_.pull_if_new(imu_mag_msg_)) {
            eskf_.fuseMag(imu_mag_msg_.mag, Eigen::Matrix3f(mag_meas_var_ * I_3));
        }

        // publish states - note that the time here is not synchronized with the filter update
        ekf_states_msg_.timestamp = current_time;
        ekf_states_msg_.position = eskf_.getStateVariable(POS_ID, 3);
        ekf_states_msg_.velocity = eskf_.getStateVariable(VEL_ID, 3);
        ekf_states_msg_.attitude  = Eigen::Quaternionf(eskf_.getStateVariable(QUAT_ID, 4).data());
        ekf_states_msg_.accel_bias = eskf_.getStateVariable(AB_ID, 3);
        ekf_states_msg_.gyro_bias = eskf_.getStateVariable(GB_ID, 3);
        ekf_states_pub_.push(ekf_states_msg_);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

} // namespace gnc