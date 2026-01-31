#ifndef ATT_CONTROL_THREAD_HPP
#define ATT_CONTROL_THREAD_HPP

#include "gnc/att_control_thread/att_control/attitude_controller.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/VehicleStateMsg.hpp"
#include "msgs/RateSetpointMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"

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

        void createAttSetpointFromRc(AttitudeSetpointMsg& attitude_setpoint);
        void controllerTask(void *pvParameters);

        // subscribers and publishers
        Topic<AttitudeSetpointMsg>::Subscriber att_setpoint_sub_;
        Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
        Topic<RcCommandMsg>::Subscriber rc_command_sub_;
        Topic<VehicleStateMsg>::Subscriber vehicle_state_sub_;
        Topic<AttitudeSetpointMsg>::Publisher att_setpoint_debug_pub_;
        Topic<RateSetpointMsg>::Publisher rate_setpoint_pub_;
        Topic<ThrustSetpointMsg>::Publisher thrust_setpoint_pub_;

        AttitudeSetpointMsg att_setpoint_debug_msg_;
        AttitudeSetpointMsg att_setpoint_msg_;
        EkfStatesMsg ekf_states_msg_;
        RcCommandMsg rc_command_msg_;
        VehicleStateMsg vehicle_state_msg_;
        RateSetpointMsg rate_setpoint_msg_;
        ThrustSetpointMsg thrust_setpoint_msg_;

        // yaw control from the RC
        float yaw_ff_gain_ = 1.0f;
        float yaw_rate_max_radps_ = 30.0f * (M_PI / 180.0f); // rad/s

        // max manual pitch roll angle
        float max_man_angle_rad_ = 25.0f * (M_PI / 180.0f); // radians

        // att controller output is in rad/s
        Eigen::Vector3f rpy_setpoint_{0.f, 0.f, NAN};
        float att_kp_[3] = {1.0f, 1.0f, 1.0f};
        float att_ki_[3] = {0.f, 0.f, 0.f};
        float att_kd_[3] = {0.f, 0.f, 0.f};
        float att_out_max_ = 100.0f * (M_PI / 180.0f);
        float att_integ_clamp_ = 0.5f;
        float att_alpha_d_[3] = {0.0f, 0.0f, 0.0f};
        float dt_ms_ = 4.0f; // 250 Hz
        Eigen::Vector3f rate_setpoint_;
        AttitudeController att_controller_;
        float max_manual_throttle_force = -20.0f; // Newtons
    };

}

#endif // ATT_CONTROL_THREAD_HPP