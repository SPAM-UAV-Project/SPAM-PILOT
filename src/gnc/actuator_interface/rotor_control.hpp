#ifndef ROTOR_CONTROL_HPP
#define ROTOR_CONTROL_HPP

#define SQ(x) ((x)*(x))

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"
#include "DShotRMT.h"
#include "pin_defs.hpp"

// msgs
#include "msgs/EncoderMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"


namespace gnc {

    class ControlAllocator
    {
    public:
        ControlAllocator() = default;
        ~ControlAllocator() = default;

        void initRotor();
    private:

        static void onRotorControlTimerEntry() {
            static_cast<ControlAllocator*>(nullptr)->onRotorControlTimer();
        }
        
        void onRotorControlTimer();

        static void allocatorTaskEntry(void* instance) {
            static_cast<ControlAllocator*>(instance)->allocatorTask(instance);
        }
        
        void allocatorTask(void *pvParameters);
        static void allocateControls(const Eigen::Vector4f& body_commands, Eigen::Vector4f& motor_forces);
        void sendToDshot(float& throttle_fraction, DShotRMT &motor);

        // Motor objects
        DShotRMT motor1_ = DShotRMT(MOTOR1_PIN, DSHOT300); // 1 motor for testing purposes
        DShotRMT motor2_ = DShotRMT(MOTOR2_PIN, DSHOT300); // Placeholder for second motor
        
        // Pubs and subs
        Topic<EncoderMsg>::Subscriber encoder_sub_;
        Topic<ThrustSetpointMsg>::Subscriber thrust_sp_sub_;
        Topic<TorqueSetpointMsg>::Subscriber torque_sp_sub_;

        // setpoint messages
        ThrustSetpointMsg thrust_sp_msg_;
        TorqueSetpointMsg torque_sp_msg_;
        EncoderMsg encoder_msg_;

        // freertos stuff
        TaskHandle_t allocator_task_handle_ = nullptr;
        static hw_timer_t* rotor_control_timer_;


    };
}

#endif // ROTOR_CONTROL_HPP