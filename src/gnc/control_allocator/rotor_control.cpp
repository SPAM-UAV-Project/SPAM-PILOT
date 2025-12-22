#include "rotor_control.hpp"
#include "sensors/encoder.hpp"

#include "DShotRMT.h"

// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

namespace control::rotor
{
    DShotRMT motor1(MOTOR1_PIN, DSHOT150); // 1 motor for testing purposes
    static float control_input[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // roll, pitch, yaw, thrust
    static SemaphoreHandle_t control_mutex = xSemaphoreCreateMutex();
    

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
        float phase = 0.0f;
        float local_control_input[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float output_throttle_fraction;

        while (true)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);            

            // copy control inputs atomically
            xSemaphoreTake(control_mutex, portMAX_DELAY);
            local_control_input[0] = control_input[0];
            local_control_input[1] = control_input[1];
            local_control_input[2] = control_input[2];
            local_control_input[3] = control_input[3];
            xSemaphoreGive(control_mutex);

            if (local_control_input[0] != 0.0f || local_control_input[1] != 0.0f)
            {
                // for swashplateless rotor control, we need to find an amplitude and a phase lag
                amplitude = AMP_OFFSET + sqrt(local_control_input[0] * local_control_input[0] + local_control_input[1] * local_control_input[1]);
                phase = atan2(local_control_input[1], local_control_input[0]);

                // convert to an oscillatory throttle response
                output_throttle_fraction = local_control_input[3] + amplitude * cos(sensors::encoder::enc_angle_rad.load() - phase);
            } else {
                // no pitch or roll command, just set throttle directly
                output_throttle_fraction = local_control_input[3];
            }
            sendToDshot(output_throttle_fraction);
        }
    }

    void setControlInputs(float roll, float pitch, float yaw, float thrust)
    {
        xSemaphoreTake(control_mutex, portMAX_DELAY);
        control_input[0] = roll;
        control_input[1] = pitch;
        control_input[2] = yaw;
        control_input[3] = thrust;
        xSemaphoreGive(control_mutex);
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