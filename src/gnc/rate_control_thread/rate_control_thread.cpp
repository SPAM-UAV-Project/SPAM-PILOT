#include "gnc/rate_control_thread/rate_control_thread.hpp"
// #include "timing/task_timing.hpp"

namespace gnc {

    RateControlThread::RateControlThread() 
    : rate_controller_(rate_kp_, rate_ki_, rate_kd_, rate_alpha_d_,
                      rate_out_max_, -rate_out_max_,
                      rate_integ_clamp_, rate_integ_clamp_,
                      dt_ms_) 
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

        while (true) {
            // task_timer.startCycle();
            // fetch data
            vehicle_state_sub_.pull_if_new(vehicle_state_msg_);

            // if not armed, then don't run the controller
            if (vehicle_state_msg_.system_state != SystemState::ARMED && vehicle_state_msg_.system_state != SystemState::ARMED_FLYING) {
                rate_controller_.reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // pull rest of the data
            rate_setpoint_sub_.pull_if_new(rate_setpoint_msg_);
            ekf_states_sub_.pull_if_new(ekf_states_msg_);
            imu_highrate_sub_.pull_if_new(imu_highrate_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);            

            // pass filtered gyro into controller
            torque_setpoint_ = inertia_matrix_.asDiagonal() * rate_controller_.run(rate_setpoint_msg_.setpoint, imu_highrate_msg_.gyro_filtered - ekf_states_msg_.gyro_bias);
            torque_setpoint_msg_.timestamp = micros();
            torque_setpoint_msg_.setpoint = torque_setpoint_;
            torque_setpoint_pub_.push(torque_setpoint_msg_);

            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 1000 == 0) {
            //     task_timer.printStats();
            // }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
}