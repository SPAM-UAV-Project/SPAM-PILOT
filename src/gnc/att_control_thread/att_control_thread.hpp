#ifndef ATT_CONTROL_THREAD_HPP
#define ATT_CONTROL_THREAD_HPP

#include "gnc/att_control_thread/att_control/attitude_controller.hpp"
#include "gnc/att_control_thread/rate_control/rate_controller.hpp"

namespace gnc {

    class AttControlThread
    {
    public:
        AttControlThread();
        ~AttControlThread() = default;

        void init();

    private:

        static void controllerEntryTask(void* instance) {
            static_cast<AttControlThread*>(instance)->controllerTask(instance);
        }

        AttitudeSetpointMsg createAttSetpointFromRc();
        void controllerTask(void *pvParameters);

        // att controller output is in rad/s
        AttitudeController att_controller_;
        Eigen::Vector3f rpy_setpoint_{0.f, 0.f, 0.f};
        float att_kp_[3] = {0.f, 0.f, 0.f};
        float att_ki_[3] = {0.f, 0.f, 0.f};
        float att_kd_[3] = {0.f, 0.f, 0.f};
        float att_out_max_ = M_PI;
        float att_integ_clamp_ = 0.5f;
        float att_alpha_d_[3] = {0.0f, 0.0f, 0.0f};
        Eigen::Vector3f rate_setpoint_;

        // yaw control from the RC
        float yaw_ff_gain_ = 1.0f;
        float yaw_rate_max_radps_ = 30.0f * (M_PI / 180.0f); // rad/s

        // max manual pitch roll angle
        float max_man_angle_rad_ = 25.0f * (M_PI / 180.0f); // radians

        // rate controller output is in normalized torque
        RateController rate_controller_;
        float rate_kp_[3] = {0.f, 0.f, 0.f};
        float rate_ki_[3] = {0.f, 0.f, 0.f};
        float rate_kd_[3] = {0.f, 0.f, 0.f};
        float rate_out_max_ = 1.0f;
        float rate_integ_clamp_ = 0.2f;
        float rate_alpha_d_[3] = {0.0f, 0.0f, 0.0f};
        Eigen::Vector3f torque_setpoint_;

        float dt_ms_ = 1.0f;

        // subscribers and publishers
        Topic<AttitudeSetpointMsg>::Subscriber att_setpoint_sub_;
        Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
        Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub_;
        Topic<RcCommandMsg>::Subscriber rc_command_sub_;
        Topic<ThrustSetpointMsg>::Subscriber thrust_setpoint_sub_;
        Topic<VehicleFlightModeMsg>::Subscriber vehicle_mode_sub_;
        Topic<ForceSetpointMsg>::Publisher force_setpoint_pub_;

        AttitudeSetpointMsg att_setpoint_msg_;
        EkfStatesMsg ekf_states_msg_;
        ImuHighRateMsg imu_highrate_msg_;
        RcCommandMsg rc_command_msg_;
        ThrustSetpointMsg thrust_setpoint_msg_;
        ForceSetpointMsg force_setpoint_msg_;
        VehicleFlightModeMsg vehicle_mode_msg_;
    };

}

#endif // ATT_CONTROL_THREAD_HPP