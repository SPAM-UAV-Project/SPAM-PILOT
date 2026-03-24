#include "gnc/vel_control_thread/vel_control_thread.hpp"
// #include "timing/task_timing.hpp"

namespace gnc {

    VelControlThread::VelControlThread() :
        vel_controller(vel_kp_, vel_ki_, vel_kd_, vel_alpha_d_,
                      vel_out_max_, -vel_out_max_,
                      vel_integ_clamp_, vel_integ_clamp_,
                      dt_ms_)
    {}

    void VelControlThread::init() {
        // initialize task
        xTaskCreate(
            controllerEntryTask,
            "Velocity Control Task",
            4096,
            this,
            2,
            nullptr
        );
    }

    void VelControlThread::controllerTask(void *pvParameters) {
        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // TaskTiming task_timer("VelControl", 20000); // 20000us budget for 50Hz

        while (true) {
            // task_timer.startCycle();
            // fetch data
            vehicle_state_sub_.pull_if_new(vehicle_state_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);
            ekf_states_sub_.pull_if_new(ekf_states_msg_);

            // if not armed, or not in proper flight mode, then don't run the controller
            if ((vehicle_state_msg_.system_state != SystemState::ARMED && vehicle_state_msg_.system_state != SystemState::ARMED_FLYING)
            || (vehicle_state_msg_.flight_mode != FlightMode::ALT_HOLD && vehicle_state_msg_.flight_mode != FlightMode::POS_HOLD)) {
                vel_controller.reset();
                rpy_setpoint_.z() = NAN; // reset yaw setpoint
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            if (!std::isfinite(rpy_setpoint_.z())) {
                // first time arming, initialize yaw setpoint to current heading to ensure it doesnt tweak out         
            }

            // convert RC to vel inputs
            createVelSpFromRC(&vel_setpoint_msg_);

            // run controller
            accel_setpoint_ = vel_controller.run(vel_setpoint_msg_.setpoint, ekf_states_msg_.velocity, ekf_states_msg_.attitude); 

            // convert accel to attitude setpoint

            // TODO: somehow convert RC yaw input to a yaw feedforward term for the attitude setpoint. Can be done in this function or before? This fills in attitude_setpoint_msg

            // publish att setpoint
            attitude_setpoint_.timestamp = micros();
            attitude_setpoint_pub_.push(attitude_setpoint_); 

            // Serial.printf("rate setpoint: %.3f, %.3f, %.3f\r\n", rate_setpoint_.x(), rate_setpoint_.y(), rate_setpoint_.z());
            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 50 == 0) {
            //     task_timer.printStats();
            // }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void VelControlThread::createVelSpFromRC(VelocitySetpointMsg *vel_setpoint_msg)
    {
        // extract roll and pitch channels to x,y vel in body frame
        vel_setpoint_msg_.setpoint.x() = rc_command_msg_.pitch * max_manual_vel_xy;
        vel_setpoint_msg_.setpoint.y() = rc_command_msg_.roll * max_manual_vel_xy;

        // z vel - create a deadzone around center, scale linearly with throttle
        // convert throttle from 0-1 to -1 to 1
        rc_command_msg_.throttle = rc_command_msg_.throttle * 2.0f - 1.0f;
        if (fabs(rc_command_msg_.throttle) < 0.1f) { 
            vel_setpoint_msg->setpoint.z() = 0.f;
        } else {
            vel_setpoint_msg->setpoint.z() = rc_command_msg_.throttle * max_manual_vel_z;
        }
    }
}