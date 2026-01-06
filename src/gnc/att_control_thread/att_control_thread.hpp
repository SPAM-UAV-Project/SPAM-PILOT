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

        void controllerTask(void *pvParameters);

        // att controller output is in rad/s
        AttitudeController att_controller_;
        float att_kp_[3] = {0.f, 0.f, 0.f};
        float att_ki_[3] = {0.f, 0.f, 0.f};
        float att_kd_[3] = {0.f, 0.f, 0.f};
        float att_out_max_ = M_PI;
        float att_integ_clamp_ = 0.5f;
        float att_alpha_d_[3] = {0.0f, 0.0f, 0.0f};
        Eigen::Vector3f rate_setpoint_;

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
    };

}

#endif // ATT_CONTROL_THREAD_HPP