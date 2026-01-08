#ifndef RATE_CONTROL_THREAD_HPP
#define RATE_CONTROL_THREAD_HPP

#include "gnc/rate_control_thread/rate_control/rate_controller.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/ForceSetpointMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/VehicleStateMsg.hpp"
#include "msgs/RateSetpointMsg.hpp"

namespace gnc {

    class RateControlThread
    {
    public:
        RateControlThread();
        ~RateControlThread() = default;
        void init();

    private:

        static void controllerEntryTask(void* instance) {
            static_cast<RateControlThread*>(instance)->controllerTask(instance);
        }

        void controllerTask(void *pvParameters);

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
        Topic<RateSetpointMsg>::Subscriber rate_setpoint_sub_;
        Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
        Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub_;
        Topic<ThrustSetpointMsg>::Subscriber thrust_setpoint_sub_;
        Topic<VehicleStateMsg>::Subscriber vehicle_state_sub_;
        Topic<RcCommandMsg>::Subscriber rc_command_sub_;
        Topic<ForceSetpointMsg>::Publisher force_setpoint_pub_;

        RateSetpointMsg rate_setpoint_msg_;
        EkfStatesMsg ekf_states_msg_;
        ImuHighRateMsg imu_highrate_msg_;
        ThrustSetpointMsg thrust_setpoint_msg_;
        VehicleStateMsg vehicle_state_msg_;
        RcCommandMsg rc_command_msg_;
        ForceSetpointMsg force_setpoint_msg_;
    };

}

#endif // RATE_CONTROL_THREAD_HPP