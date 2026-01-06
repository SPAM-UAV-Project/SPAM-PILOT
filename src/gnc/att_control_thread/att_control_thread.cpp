#include "gnc/att_control_thread/att_control_thread.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"

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
        
        // subscribers and publishers
        Topic<AttitudeSetpointMsg>::Subscriber att_setpoint_sub_;
        Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
        Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub_;
        Topic<TorqueSetpointMsg>::Publisher torque_setpoint_pub_;

        AttitudeSetpointMsg att_setpoint_msg_;
        EkfStatesMsg ekf_states_msg_;
        ImuHighRateMsg imu_highrate_msg_;
        TorqueSetpointMsg torque_setpoint_msg_;

        const TickType_t xFrequency = pdMS_TO_TICKS(static_cast<TickType_t>(dt_ms_));
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // run attitude loop at 250 Hz, rate at 1000 Hz 
        const int att_decimation = 4;
        int att_counter = 0;

        while (true) {
            // obtain data
            att_setpoint_sub_.pull_if_new(att_setpoint_msg_);
            ekf_states_sub_.pull_if_new(ekf_states_msg_);
            imu_highrate_sub_.pull_if_new(imu_highrate_msg_); // probably add a lpf on this later
            
            // decimate attitude controller
            if (att_counter == 0) {
                rate_setpoint_ = att_controller_.run(att_setpoint_msg_.setpoint, ekf_states_msg_.attitude);
            }
            att_counter = (att_counter + 1) % att_decimation;
                    
            // rate controller
            torque_setpoint_ = rate_controller_.run(rate_setpoint_, imu_highrate_msg_.gyro - ekf_states_msg_.gyro_bias);

            // control allocation
            // send motor forces setpoint

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

}