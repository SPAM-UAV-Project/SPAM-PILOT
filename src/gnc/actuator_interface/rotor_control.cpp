#include "rotor_control.hpp"
#include "sensors/encoder/encoder.hpp"
#include "DShotRMT.h"
#include "msgs/EncoderMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"

// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

namespace control::rotor
{
    DShotRMT motor1(MOTOR1_PIN, DSHOT150); // 1 motor for testing purposes
    static float control_input[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // roll, pitch, yaw, thrust
    static SemaphoreHandle_t control_mutex = xSemaphoreCreateMutex();
    
    // subscribers
    Topic<EncoderMsg>::Subscriber encoder_sub;
    Topic<ThrustSetpointMsg>::Subscriber thrust_sp_sub;
    Topic<TorqueSetpointMsg>::Subscriber torque_sp_sub;

    // setpoint messages
    ThrustSetpointMsg thrust_sp_msg;
    TorqueSetpointMsg torque_sp_msg;
    EncoderMsg encoder_msg;


    // timer interrupts
    static hw_timer_t* rotorControlTimer = NULL;

    // control timer interrupt for precise timing
    void IRAM_ATTR onRotorControlTimer() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // notify the rotor control task to run
        vTaskNotifyGiveFromISR(rotorTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    
    void initRotor()
    {
        motor1.begin();
        motor1.sendThrottle(0);

        Serial.println("[Rotor Controller]: Initializing rotor control...");
        for(int i = 0; i < 300; i++) {  // 3 seconds of zero throttle
            delay(10);
        }

        xTaskCreatePinnedToCore(rotorControlTask, "RotorControlTask", 4096, NULL, 3, &rotorTaskHandle, 0);
    
        // create timer
        Serial.println("[Rotor Controller]: Setting up rotor control timer...");
        rotorControlTimer = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(rotorControlTimer, &onRotorControlTimer);
        timerAlarm(rotorControlTimer, 1000, true, 0); // 1000 Hz alarm, auto-reload
        Serial.println("[Rotor Controller]: Rotor control initialized.");
    }

    void rotorControlTask(void *pvParameters)
    {
        float amplitude = 0.0f;
        float amp_cut_in = 0.16f;
        float phase = 0.0f;
        float phase_lag = M_PI / 6.0f; // 30 degrees phase lag
        float output_throttle_fraction;

        Eigen::Vector4f motor_forces = Eigen::Vector4f::Zero(); // f1x, f1y, f1z, f2z
        Eigen::Vector4f body_commands = Eigen::Vector4f::Zero(); // thrust, torque_x, torque_y, torque_z
        Eigen::Matrix4f allocation_matrix;
        
        while (true)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);            

            // receive data
            encoder_sub.pull_if_new(encoder_msg);
            thrust_sp_sub.pull_if_new(thrust_sp_msg);
            if (torque_sp_sub.pull_if_new(torque_sp_msg)){
                // allocate motor commands
                body_commands << thrust_sp_msg.setpoint, torque_sp_msg.setpoint.x(), torque_sp_msg.setpoint.y(), torque_sp_msg.setpoint.z();
                // map to motor forces
                motor_forces = allocation_matrix * body_commands;
            }

            if (motor_forces[0] != 0.0f || motor_forces[1] != 0.0f)
            {
                // for swashplateless rotor control, we need to find an amplitude and a phase lag
                amplitude = amp_cut_in + sqrt(motor_forces[0] * motor_forces[0] + motor_forces[1] * motor_forces[1]);
                phase = atan2(motor_forces[1], motor_forces[0]);

                // convert to an oscillatory throttle response
                output_throttle_fraction = ((motor_forces[3]) - motor_forces[2]) + amplitude * cos(encoder_msg.angle_rad - phase - phase_lag);
                // clamp output from arming throttle to 100%
                // print output and encoder
                //output_throttle_fraction = std::max(ARMING_THROTTLE, std::min(1.0f, output_throttle_fraction));
            } else {
                // no pitch or roll command, just set throttle directly
                output_throttle_fraction = ((motor_forces[3]) - motor_forces[2]); // to do fix yaw allocation and add second dshot output
            }
            sendToDshot(output_throttle_fraction);
        }
    }
    
    void sendToDshot(float throttle_fraction)
    { 
        if (throttle_fraction <= 0.0f) {
            
            return;
        }
        // ensure throttle_fraction is within [0.0, 1.0]
        throttle_fraction = std::max(0.0f, std::min(1.0f, throttle_fraction));
        // convert to DShot value (48 to 2047 for throttle)
        uint16_t dshot_value = static_cast<uint16_t>(48 + throttle_fraction * (2047 - 48));
        motor1.sendThrottle(dshot_value);
    }
}