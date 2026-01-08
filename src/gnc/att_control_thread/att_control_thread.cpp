#include "gnc/att_control_thread/att_control_thread.hpp"

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
            3,
            nullptr
        );
    }

    void AttControlThread::controllerTask(void *pvParameters) {
        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        while (true) {
            // fetch data
            vehicle_state_sub_.pull_if_new(vehicle_state_msg_);

            // if not armed, then don't run the controller
            if (vehicle_state_msg_.system_state != SystemState::ARMED && vehicle_state_msg_.system_state != SystemState::ARMED_FLYING) {
                att_controller_.reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // pull rest of the data
            ekf_states_sub_.pull_if_new(ekf_states_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);
            att_setpoint_sub_.pull_if_new(att_setpoint_msg_);

            if (vehicle_state_msg_.flight_mode == FlightMode::STABILIZED ||
                vehicle_state_msg_.flight_mode == FlightMode::ALT_HOLD) {
                createAttSetpointFromRc(att_setpoint_msg_); // override with rc command in manual modes
            }
            
            rate_setpoint_ = att_controller_.run(att_setpoint_msg_.q_sp, ekf_states_msg_.attitude);
            rate_setpoint_.z() += yaw_ff_gain_ * att_setpoint_msg_.yaw_sp_ff_rate;

            // publish rate setpoint
            rate_setpoint_msg_.timestamp = micros();
            rate_setpoint_msg_.setpoint = rate_setpoint_;
            rate_setpoint_pub_.push(rate_setpoint_msg_);

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void AttControlThread::createAttSetpointFromRc(AttitudeSetpointMsg& attitude_setpoint) {
        // create attitude setpoint quaternion from rc command roll, pitch, yaw angles

        rpy_setpoint_.x() = rc_command_msg_.roll * max_man_angle_rad_;
        rpy_setpoint_.y() = rc_command_msg_.pitch * max_man_angle_rad_;
        rpy_setpoint_.z() += rc_command_msg_.yaw * yaw_rate_max_radps_ * (dt_ms_ / 1000.0f);

        wrapAngle(rpy_setpoint_.z());

        attitude_setpoint.q_sp = Eigen::AngleAxisf(rpy_setpoint_.z(), Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(rpy_setpoint_.y(), Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(rpy_setpoint_.x(), Eigen::Vector3f::UnitX());
        attitude_setpoint.yaw_sp_ff_rate = rc_command_msg_.yaw * yaw_rate_max_radps_;
    }

}