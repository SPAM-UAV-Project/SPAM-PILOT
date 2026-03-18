#ifndef VEL_CONTROL_THREAD_HPP
#define VEL_CONTROL_THREAD_HPP

#include "gnc/vel_control_thread/vel_control/vel_controller.hpp"
#include "msgs/VelocitySetpointMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/VehicleStateMsg.hpp"
#include "msgs/RateSetpointMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"

namespace gnc {

    class VelControlThread
    {
    public:
        VelControlThread();
        ~VelControlThread() = default;

        void init();

    private:

        static void controllerEntryTask(void* instance) {
            static_cast<VelControlThread*>(instance)->controllerTask(instance);
        }

        void controllerTask(void *pvParameters);

        // subscribers and publishers
        Topic<VelocitySetpointMsg>::Subscriber vel_setpoint_sub_;
        Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
        Topic<RcCommandMsg>::Subscriber rc_command_sub_;
        Topic<VehicleStateMsg>::Subscriber vehicle_state_sub_;
        Topic<VelocitySetpointMsg>::Publisher vel_setpoint_debug_pub_; // this must NOT be published if vel_setpoint_sub is being called
        Topic<RateSetpointMsg>::Publisher rate_setpoint_pub_;
        Topic<ThrustSetpointMsg>::Publisher thrust_setpoint_pub_;

        VelocitySetpointMsg vel_setpoint_msg_;
        EkfStatesMsg ekf_states_msg_;
        RcCommandMsg rc_command_msg_;
        VehicleStateMsg vehicle_state_msg_;
        RateSetpointMsg rate_setpoint_msg_;
        ThrustSetpointMsg thrust_setpoint_msg_;

        VelController vel_controller;
        float vel_kp_[3] = {1.5f, 1.5f, 0.75f};
        float vel_ki_[3] = {0.f, 0.f, 0.f};
        float vel_kd_[3] = {0.f, 0.f, 0.f};
        float vel_out_max_ = 100.0f * (M_PI / 180.0f); // max accel setpoint
        float vel_integ_clamp_ = 0.5f;
        float vel_alpha_d_[3] = {0.0f, 0.0f, 0.0f};
        Eigen::Vector3f rpy_setpoint_{0.f, 0.f, NAN};
        Eigen::Quaternionf attitude_setpoint_;

        float max_manual_throttle_force = -20.0f; // Newtons

        // yaw control from the RC
        float yaw_ff_gain_ = 1.0f;
        float yaw_rate_max_radps_ = 40.0f * (M_PI / 180.0f); // rad/s

        float dt_ms_ = 10.0f; // ms - 100 Hz


    };

}

#endif // VEL_CONTROL_THREAD_HPP