#ifndef ROTOR_CONTROL_HPP
#define ROTOR_CONTROL_HPP

#define MOTOR1_PIN 20
#define AMP_OFFSET 0.0f //  amplitude offset to overcome static friction of hinge

#include <Arduino.h>

namespace control::rotor
{
    inline TaskHandle_t rotorTaskHandle = NULL;

    void initRotor();
    void rotorControlTask(void *pvParameters);
    void setControlInputs(float roll, float pitch, float yaw, float thrust);
    void sendToDshot(float throttle_percent);
}

#endif // ROTOR_CONTROL_HPP