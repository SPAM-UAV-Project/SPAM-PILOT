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

    int imu_hr_samples = 0; 
    int imu_mag_samples = 200; // disable mag for now

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
    init_gyro_bias_ = gyro_sum / 250.0f;
    Eigen::Vector3f avg_accel = accel_sum / 250.0f;
    Eigen::Vector3f avg_mag = mag_sum / 250.0f;

    Serial.printf("Initial Gyro Bias: [%.4f, %.4f, %.4f]\n", init_gyro_bias_.x(), init_gyro_bias_.y(), init_gyro_bias_.z());


    // guess initial orientation - TO DO - maybe get from triad with accel and mag
    init_quat_ << 0.0f, 0.0f, 0.0f, 1.0f; // x, y, z, w
}

void StateEstimator::init() 
{
    getInitialStates();

    // initialize eskf class 
    eskf_.initStates(Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(),
                     init_quat_,
                     Eigen::Vector3f::Zero(),
                     init_gyro_bias_);
    eskf_.initCovariances(I_3 * pos_var_init_, I_3 * vel_var_init_,
                          I_3 * attitude_var_init_, I_3 * ab_var_init_,
                          I_3 * gb_var_init_);
    eskf_.initSensorNoise(accel_noise_var_, accel_walk_var_,
                          gyro_noise_var_, gyro_walk_var_);

    // start the estimator
    xTaskCreatePinnedToCore(
        StateEstimator::stateEstimatorTaskEntry,
        "StateEstimatorTask",
        8192,
        this,
        3,
        &stateEstimatorTaskHandle_,
        1
    );
    
}

void StateEstimator::stateEstimatorTask(void *pvParameters) 
{
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    long last_loop_time = micros();

    while (true) {
        // obtain sensor data
        if (!imu_gyro_accel_sub_.pull_if_new(imu_gyro_accel_msg_)) {
            continue; // probably failsafe here if lost gyro data
        }

        // compute dt
        long current_time = micros();
        dt_ = (current_time - last_loop_time) * 1e-6f; // convert to seconds
        last_loop_time = current_time;

        // obtain prior estimate
        eskf_.predictStates(imu_gyro_accel_msg_.accel,
                                imu_gyro_accel_msg_.gyro,
                                dt_);

        // publish prior state estimate for now
        ekf_states_msg_.timestamp = current_time;
        ekf_states_msg_.position = eskf_.getStateVariable(POS_ID, 3);
        ekf_states_msg_.velocity = eskf_.getStateVariable(VEL_ID, 3);
        ekf_states_msg_.attitude = eskf_.getStateVariable(QUAT_ID, 4);
        ekf_states_msg_.accel_bias = eskf_.getStateVariable(AB_ID, 3);
        ekf_states_msg_.gyro_bias = eskf_.getStateVariable(GB_ID, 3);
        ekf_states_pub_.push(ekf_states_msg_);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

} // namespace gnc