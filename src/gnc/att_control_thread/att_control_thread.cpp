#include "gnc/att_control_thread/att_control_thread.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/ForceSetpointMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/VehicleFlightModeMsg.hpp"

namespace gnc {

    AttControlThread::AttControlThread() 
    : att_controller_(att_kp_, att_ki_, att_kd_, att_alpha_d_,
                      att_out_max_, -att_out_max_,
                      att_integ_clamp_, att_integ_clamp_,
                      dt_ms_), 
      rate_controller_(rate_kp_, rate_ki_, rate_kd_, rate_alpha_d_,
                      rate_out_max_, -rate_out_max_,
                      rate_integ_clamp_, rate_integ_clamp_,
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
            4,
            nullptr
        );
    }

    void AttControlThread::controllerTask(void *pvParameters) {
        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // run attitude loop at 250 Hz, rate at 1000 Hz 
        const int att_decimation = 4;
        int att_counter = 0;

        while (true) {
            // fetch data
            ekf_states_sub_.pull_if_new(ekf_states_msg_);
            imu_highrate_sub_.pull_if_new(imu_highrate_msg_); // probably add a lpf on this later
            vehicle_mode_sub_.pull_if_new(vehicle_mode_msg_);
            rc_command_sub_.pull_if_new(rc_command_msg_);
            att_setpoint_sub_.pull_if_new(att_setpoint_msg_);

            // att controller
            if (att_counter == 0) {
                if (vehicle_mode_msg_.flight_mode == FlightMode::STABILIZED ||
                    vehicle_mode_msg_.flight_mode == FlightMode::ALT_HOLD) {
                    att_setpoint_msg_ = createAttSetpointFromRc(); // override with rc command in manual modes
                }
                rate_setpoint_ = att_controller_.run(att_setpoint_msg_.q_sp, ekf_states_msg_.attitude);
                rate_setpoint_.z() += yaw_ff_gain_ * att_setpoint_msg_.yaw_sp_ff_rate;
            }
            att_counter = (att_counter + 1) % att_decimation;
                    
            // rate controller // may move into own task, this loop is getting heavy
            torque_setpoint_ = rate_controller_.run(rate_setpoint_, imu_highrate_msg_.gyro - ekf_states_msg_.gyro_bias);

            // perform control allocation and then send to actuator interface
            thrust_setpoint_sub_.pull_if_new(thrust_setpoint_msg_);
            force_setpoint_msg_.timestamp = micros();

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    AttitudeSetpointMsg AttControlThread::createAttSetpointFromRc() {
        // create attitude setpoint quaternion from rc command roll, pitch, yaw angles
        AttitudeSetpointMsg attitude_setpoint;

        rpy_setpoint_.x() = rc_command_msg_.roll * max_man_angle_rad_;
        rpy_setpoint_.y() = rc_command_msg_.pitch * max_man_angle_rad_;
        rpy_setpoint_.z() += rc_command_msg_.yaw * yaw_rate_max_radps_ * (4.0f / 1000.0f); // assuming this is called at 250 Hz

        wrapAngle(rpy_setpoint_.z());

        attitude_setpoint.q_sp = Eigen::AngleAxisf(rpy_setpoint_.z(), Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(rpy_setpoint_.y(), Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(rpy_setpoint_.x(), Eigen::Vector3f::UnitX());
        attitude_setpoint.yaw_sp_ff_rate = rc_command_msg_.yaw * yaw_rate_max_radps_;
        return attitude_setpoint;
    }

}