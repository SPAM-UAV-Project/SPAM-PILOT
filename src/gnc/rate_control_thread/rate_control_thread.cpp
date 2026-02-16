#include "gnc/rate_control_thread/rate_control_thread.hpp"
#include "filter/butter_lp.hpp"
#include "models/second_order_actuator.hpp"
#include "timing/task_timing.hpp"


namespace gnc {

    RateControlThread::RateControlThread() 
    // : rate_controller_(rate_kp_, rate_ki_, rate_kd_, rate_alpha_d_,
    //                   rate_out_max_, -rate_out_max_,
    //                   rate_integ_clamp_, rate_integ_clamp_,
    //                   dt_ms_) 
    : indi_controller_(G_effectiveness_)
    {}

    void RateControlThread::init() {
        // initialize task
        xTaskCreate(
            controllerEntryTask,
            "Rate Control Task",
            4096,
            this,
            2,
            nullptr
        );
    }

    void RateControlThread::controllerTask(void *pvParameters) {
        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // TaskTiming task_timer("RateControl", 1000); // 1000us budget for 1kHz
        
        // vars for controller
        Eigen::Vector3f prev_ang_vel = Eigen::Vector3f::Zero();
        Eigen::Vector3f ang_accel = Eigen::Vector3f::Zero();    
        float dt_s = dt_ms_ * 0.001f;

        // butter low pass filter for angular acceleration
        ButterLowPassFilt ang_accel_filt_;
        ang_accel_filt_.setup(20.0f, 1000.0f); // 20 Hz cutoff, 1 kHz sample rate
        ButterLowPassFilt torque_delay_filt_;
        torque_delay_filt_.setup(20.0f, 1000.0f); // 20 Hz cutoff, 1 kHz sample rate
        
        // actuator model
        SecondOrderActuator actuator_model_;
        auto xy_coeffs = SecondOrderActuator::discretize(-0.87379f,71.419f, 13.133f,72.644f, dt_s);
        auto z_coeffs = SecondOrderActuator::discretize(1.8273f,59.457f, 12.522f,59.457f, dt_s);
        actuator_model_.setup(xy_coeffs, xy_coeffs, z_coeffs);

        while (true) {
            // task_timer.startCycle();
            // fetch data
            vehicle_state_sub_.pull_if_new(vehicle_state_msg_);

            // pull rest of the data
            rate_setpoint_sub_.pull_if_new(rate_setpoint_msg_);
            ekf_states_sub_.pull_if_new(ekf_states_msg_);
            imu_highrate_sub_.pull_if_new(imu_highrate_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);           
            
            // get angular acceleration
            Eigen::Vector3f ang_vel = imu_highrate_msg_.gyro_filtered - ekf_states_msg_.gyro_bias;
            Eigen::Vector3f ang_accel_raw = (ang_vel - prev_ang_vel) / dt_s;
            ang_accel_filt_.apply3d(ang_accel_raw.data(), ang_accel.data());
            prev_ang_vel = ang_vel;

            // publish angular acceleration
            ang_accel_msg_.timestamp = micros();
            ang_accel_msg_.ang_accel_raw = ang_accel_raw;
            ang_accel_msg_.ang_accel_filtered = ang_accel;
            ang_accel_pub_.push(ang_accel_msg_);

            // if not armed, then don't run the controller
            if ((vehicle_state_msg_.system_state != SystemState::ARMED && vehicle_state_msg_.system_state != SystemState::ARMED_FLYING) 
                || rc_command_msg_.throttle < 0.25f) {
                // rate_controller_.reset();
                delayed_torque_setpoint_.setZero();
                torque_setpoint_.setZero();
                actuator_model_.reset();
                torque_delay_filt_.reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // run INDI control algorithm
            Eigen::Vector3f virtual_accel_input = virtual_p_gain_ * (rate_setpoint_msg_.setpoint - ang_vel);
            Eigen::Vector3f indi_increment = indi_controller_.runDeltaU(virtual_accel_input, ang_accel);
            torque_setpoint_ = delayed_torque_setpoint_ + indi_increment;
            
            torque_setpoint_msg_.timestamp = ang_accel_msg_.timestamp;
            torque_setpoint_msg_.setpoint = torque_setpoint_;
            torque_setpoint_msg_.delayed_torque = delayed_torque_setpoint_;
            torque_setpoint_msg_.indi_increment = indi_increment;
            torque_setpoint_pub_.push(torque_setpoint_msg_);

            // delay the torque setpoint by actuator delay and filter delay

            // delay torque by second order actuator model
            delayed_torque_setpoint_ = actuator_model_.update(torque_setpoint_);

            // delay by accel filt delay (use temp to avoid in-place aliasing)
            Eigen::Vector3f temp;
            torque_delay_filt_.apply3d(delayed_torque_setpoint_.data(), temp.data());
            delayed_torque_setpoint_ = temp;

            // task_timer.endCycle();

            // if (task_timer.getCycleCount() % 1000 == 0) {
            //     // print torque_setpoint in raw 
            //     // Serial.printf("Torque setpoint (raw): [%.2f, %.2f, %.2f] mNm\n", torque_setpoint_.x() * 1e3f, torque_setpoint_.y() * 1e3f, torque_setpoint_.z() * 1e3f);
            //     task_timer.printStats();
            // }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
}