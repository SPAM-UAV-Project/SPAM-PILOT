#include "gnc/att_control_thread/att_control_thread.hpp"
// #include "timing/task_timing.hpp"

namespace gnc {

    AttControlThread::AttControlThread() 
    : att_controller_(att_kp_, att_ki_, att_kd_, att_alpha_d_,
                      att_out_max_, -att_out_max_,
                      att_integ_clamp_, att_integ_clamp_,
                      dt_ms_)
    {}

    void wrapAngle(float& angle) {
        if (angle > M_PI) {
            angle -= 2.0f * M_PI;
        } else if (angle < -M_PI) {
            angle += 2.0f * M_PI;
        }
    }

    void AttControlThread::init() {
        // initialize task
        xTaskCreate(
            controllerEntryTask,
            "Attitude Control Task",
            4096,
            this,
            2,
            nullptr
        );
    }

    void AttControlThread::controllerTask(void *pvParameters) {
        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // TaskTiming task_timer("AttControl", 20000); // 20000us budget for 50Hz

        while (true) {
            // task_timer.startCycle();
            // fetch data
            vehicle_state_sub_.pull_if_new(vehicle_state_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);
            ekf_states_sub_.pull_if_new(ekf_states_msg_);

            // if not armed, then don't run the controller
            if (vehicle_state_msg_.system_state != SystemState::ARMED && vehicle_state_msg_.system_state != SystemState::ARMED_FLYING) {
                att_controller_.reset();
                rpy_setpoint_.z() = NAN; // reset yaw setpoint
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            if (!std::isfinite(rpy_setpoint_.z())) {
                // first time arming, initialize yaw setpoint to current heading
                Eigen::Quaternionf q = ekf_states_msg_.attitude;
                rpy_setpoint_.z() = atan2f(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                                                1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));            
            }

            // choose setpoints based on flight mode
            switch (vehicle_state_msg_.flight_mode)
            {
            case FlightMode::STABILIZED: // subs: rc_cmd | pubs: thrust + att sp
                createAttSetpointFromRc(att_setpoint_msg_); // overwrite with rc command in manual modes
                att_setpoint_msg_.timestamp = micros();
                att_setpoint_debug_pub_.push(att_setpoint_msg_);

                // thrust setpoint from rc (0 to 1 mapped to 0 to 10 N)
                thrust_setpoint_msg_.setpoint = max_manual_throttle_force * rc_command_msg_.throttle; // map directly for now (adjust expo in rc controller)
                thrust_setpoint_pub_.push(thrust_setpoint_msg_);
                break;
                
            case FlightMode::ALT_HOLD: // subs: rc_cmd | pubs: att sp
                createAttSetpointFromRc(att_setpoint_msg_); // overwrite with rc command in manual modes
                att_setpoint_msg_.timestamp = micros();
                att_setpoint_debug_pub_.push(att_setpoint_msg_);
                break;
            case FlightMode::POS_HOLD: // subs: att_sp | pubs: none
                att_setpoint_sub_.pull_if_new(att_setpoint_msg_);
                break;
            default:
                break;
            }

            // run controller
            rate_setpoint_ = att_controller_.run(att_setpoint_msg_.q_sp, ekf_states_msg_.attitude);
            rate_setpoint_.z() += yaw_ff_gain_ * att_setpoint_msg_.yaw_sp_ff_rate;

            // publish rate setpoint
            rate_setpoint_msg_.timestamp = micros();
            rate_setpoint_msg_.setpoint = rate_setpoint_;
            rate_setpoint_pub_.push(rate_setpoint_msg_);

            // Serial.printf("rate setpoint: %.3f, %.3f, %.3f\r\n", rate_setpoint_.x(), rate_setpoint_.y(), rate_setpoint_.z());
            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 50 == 0) {
            //     task_timer.printStats();
            // }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void AttControlThread::createAttSetpointFromRc(AttitudeSetpointMsg& attitude_setpoint) {
        // create attitude setpoint quaternion from rc command roll, pitch, yaw angles

        rpy_setpoint_.x() = rc_command_msg_.roll * max_man_angle_rad_;
        rpy_setpoint_.y() = rc_command_msg_.pitch * max_man_angle_rad_;
        rpy_setpoint_.z() += rc_command_msg_.yaw * yaw_rate_max_radps_ * dt_ms_ * 0.001f;

        wrapAngle(rpy_setpoint_.z());

        attitude_setpoint.q_sp = Eigen::AngleAxisf(rpy_setpoint_.z(), Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(rpy_setpoint_.y(), Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(rpy_setpoint_.x(), Eigen::Vector3f::UnitX());
        attitude_setpoint.yaw_sp_ff_rate = rc_command_msg_.yaw * yaw_rate_max_radps_;
    }

}