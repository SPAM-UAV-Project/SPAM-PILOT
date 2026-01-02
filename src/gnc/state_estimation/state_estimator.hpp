/**
 * @file state_estimator.hpp
 * @brief header file for the state estimation thread and associated functions
 */

#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include "gnc/state_estimation/ESKF.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ImuMagMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include <Arduino.h>

namespace gnc {

    class StateEstimator
    {
    public:
        StateEstimator();
        ~StateEstimator() = default;
        void init();
        static void stateEstimatorTaskEntry(void* instance) {
            static_cast<StateEstimator*>(instance)->stateEstimatorTask(instance);
        }

    private:
        void stateEstimatorTask(void *pvParameters);
        void getInitialStates();

        xTaskHandle stateEstimatorTaskHandle_ = NULL;

        ESKF eskf_;
        float dt_ = 0.004f; // 250 Hz

        // initial conditions
        Eigen::Vector4f init_quat_;
        Eigen::Vector3f init_LLA_;
        Eigen::Vector3f init_mag_nav_; // initial mag in nav frame
        Eigen::Vector3f init_accel_bias_;
        Eigen::Vector3f init_gyro_bias_;
        float pos_var_init_ = 100.f, vel_var_init_ = 100.f, attitude_var_init_ = 0.5f;
        float ab_var_init_ = 0.5f, gb_var_init_ = 0.3f;

        // sensor noise from datasheet
        float accel_noise_var_ = 0.1f; // (m/s^2)^2/Hz
        float accel_walk_var_ = 0.004f; // (m/s^2)^2*Hz
        float gyro_noise_var_ = 0.016f; // (rad/s)^2/Hz
        float gyro_walk_var_ = 0.004f; // (rad/s)^2*Hz

        // subscribers
        Topic<ImuHighRateMsg>::Subscriber imu_gyro_accel_sub_;
        Topic<ImuMagMsg>::Subscriber imu_mag_sub_;

        ImuHighRateMsg imu_gyro_accel_msg_;
        ImuMagMsg imu_mag_msg_;

        // publishers
        Topic<EkfStatesMsg>::Publisher ekf_states_pub_;
        EkfStatesMsg ekf_states_msg_;

    };    
}
#endif // STATE_ESTIMATOR_HPP

