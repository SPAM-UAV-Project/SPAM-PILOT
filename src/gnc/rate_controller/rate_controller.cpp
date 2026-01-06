#include "gnc/rate_controller/rate_controller.hpp"
#include "msgs/RateSetpointMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"

namespace gnc {

RateController::RateController() : pid_x_(dt_ / 1000.0f, out_max_, out_min_, 1.0f, kp_[0], ki_[0], kd_[0], integ_clamp_xy_, alpha_d_[0]),
                                   pid_y_(dt_ / 1000.0f, out_max_, out_min_, 1.0f, kp_[1], ki_[1], kd_[1], integ_clamp_xy_, alpha_d_[1]),
                                   pid_z_(dt_ / 1000.0f, out_max_, out_min_, 1.0f, kp_[2], ki_[2], kd_[2], integ_clamp_z_, alpha_d_[2])
{}

Eigen::Vector3f RateController::run(Eigen::Vector3f rate_setpoint, Eigen::Vector3f rate_measurement)
{
    return {
        pid_x_.run(rate_setpoint[0], rate_measurement[0]),
        pid_y_.run(rate_setpoint[1], rate_measurement[1]),
        pid_z_.run(rate_setpoint[2], rate_measurement[2])
    };
}

void RateController::init()
{
    xTaskCreate(
        rateControlTaskEntry,
        "RateControlTask",
        2048,
        this,
        4,
        nullptr
    );
}

void RateController::rateControlTask(void *pvParameters)
{
    // subs and pubs
    Topic<RateSetpointMsg>::Subscriber rate_setpoint_sub;
    Topic<TorqueSetpointMsg>::Publisher torque_setpoint_pub;
    Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub;
    Topic<EkfStatesMsg>::Subscriber ekf_states_sub;

    RateSetpointMsg rate_setpoint_msg;
    TorqueSetpointMsg torque_setpoint_msg;
    ImuHighRateMsg imu_highrate_msg;
    EkfStatesMsg ekf_states_msg;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {

        // get rate setpoint, gyro, and ekf biases
        rate_setpoint_sub.pull_if_new(rate_setpoint_msg);
        imu_highrate_sub.pull_if_new(imu_highrate_msg);
        ekf_states_sub.pull_if_new(ekf_states_msg);

        torque_setpoint_msg.setpoint = run(rate_setpoint_msg.rate_setpoint, imu_highrate_msg.gyro - ekf_states_msg.gyro_bias);
        torque_setpoint_msg.timestamp = rate_setpoint_msg.timestamp;

        // publish setpoints
        torque_setpoint_pub.push(torque_setpoint_msg);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(dt_));
    }

}

};