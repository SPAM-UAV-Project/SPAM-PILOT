/**
 * @file state_estimator.cpp
 * @brief Implementation of the state estimation thread for estimating vehicle position and attitude
 */

#include "gnc/state_estimation/state_estimator.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ImuIntegrated.hpp"
#include "msgs/ImuMagMsg.hpp"

#include "timing/task_timing.hpp"

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
            // Serial.printf("[StateEstimator] Collecting Mag Sample %d: [%.4f, %.4f, %.4f] uT\n", (int)imu_mag_samples,
            //               imu_mag_msg_.mag.x(), imu_mag_msg_.mag.y(), imu_mag_msg_.mag.z());
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

    // initialize accel meas filter
    accel_meas_filter_.setTimeConst(0.05f, dt_);
    accel_meas_filtered_ = accel_meas_filter_.apply3d(avg_accel);

    eskf_.initStates(Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(),
                     init_quat_,
                     Eigen::Vector3f::Zero(),
                     init_gyro_bias_,
                     init_mag_nav_);
    eskf_.initCovariances(I_3 * pos_var_init_, I_3 * vel_var_init_,
                          I_3 * attitude_var_init_, I_3 * ab_var_init_,
                          I_3 * gb_var_init_);
    
}

void StateEstimator::init() 
{
    // initialize eskf class 
    eskf_.initSensorNoise(accel_noise_var_, accel_walk_var_,
                          gyro_noise_var_, gyro_walk_var_);

    // start the estimator
    xTaskCreatePinnedToCore(
        StateEstimator::stateEstimatorTaskEntry,
        "StateEstimatorTask",
        16384,
        this,
        3,
        &stateEstimatorTaskHandle_,
        0
    );

    Serial.println("[StateEstimator] State Estimator Task Started.");
}

void StateEstimator::stateEstimatorTask(void *pvParameters) 
{
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    long last_loop_time = micros();
    long current_time = micros();
    uint32_t loop_counter = 0;

    TaskTiming task_timer("StateEstimator", 4000); // 4000 us budget for 250 hz

    // gravity fusion temp vars
    Eigen::Vector3f delta_vel_meas_sum = Eigen::Vector3f::Zero();
    float delta_vel_dt_sum = 0.f;
    constexpr uint32_t grav_fusion_rate = 5;

    getInitialStates();

    while (true) {
        // start timer
        task_timer.startCycle();

        // obtain sensor data
        if (imu_integrated_sub_.pull_if_new(imu_integrated_msg_)) {
            // compute dt
            current_time = micros();
            dt_ = (current_time - last_loop_time) * 1e-6f; // convert to seconds
            last_loop_time = current_time;

            // obtain prior estimate
            eskf_.predictStates(imu_integrated_msg_);

            // fuse gravity 
            accel_meas_filtered_ = accel_meas_filter_.apply3d(imu_integrated_msg_.delta_vel / imu_integrated_msg_.delta_vel_dt);
            delta_vel_dt_sum += imu_integrated_msg_.delta_vel_dt;
            delta_vel_meas_sum += imu_integrated_msg_.delta_vel;
            if (loop_counter % grav_fusion_rate == 0) { // fuse gravity at 50 Hz to save computation
                if (delta_vel_dt_sum > 0) {
                    eskf_.fuseGravity(delta_vel_meas_sum / delta_vel_dt_sum, accel_meas_filtered_, accel_noise_var_ / grav_fusion_rate);
                    delta_vel_meas_sum.setZero();
                    delta_vel_dt_sum = 0.f;
                }
            }
            loop_counter++;
        }

        // if received mag data, update the filter
        if (imu_mag_sub_.pull_if_new(imu_mag_msg_)) {
            eskf_.fuseMag(imu_mag_msg_.mag, mag_meas_var_);
            // publish innovations on mag fusion (slowest)
            eskf_.ekf_innovations_msg.timestamp = current_time;
            eskf_.ekf_innovations_pub.push(eskf_.ekf_innovations_msg);
        }

        // publish states - note that the time here is not synchronized with the filter update
        ekf_states_msg_.timestamp = current_time;
        ekf_states_msg_.position = eskf_.getStateVariable(POS_ID, 3);
        ekf_states_msg_.velocity = eskf_.getStateVariable(VEL_ID, 3);
        ekf_states_msg_.attitude  = Eigen::Quaternionf(eskf_.getStateVariable(QUAT_ID, 4).data());
        ekf_states_msg_.accel_bias = eskf_.getStateVariable(AB_ID, 3);
        ekf_states_msg_.gyro_bias = eskf_.getStateVariable(GB_ID, 3);
        ekf_states_pub_.push(ekf_states_msg_);

        // Serial.printf("gyro biases: [%.4f, %.4f, %.4f] rad/s\n", ekf_states_msg_.gyro_bias.x(), ekf_states_msg_.gyro_bias.y(), ekf_states_msg_.gyro_bias.z());
        // Serial.printf("accel_biases: [%.4f, %.4f, %.4f] m/s^2\n", ekf_states_msg_.accel_bias.x(), ekf_states_msg_.accel_bias.y(), ekf_states_msg_.accel_bias.z());
        // print timing info
        // task_timer.endCycle();

        // if (task_timer.getCycleCount() % 250 == 0) {
        //     task_timer.printStats();
        // }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

} // namespace gnc