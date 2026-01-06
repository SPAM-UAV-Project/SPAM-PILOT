#ifndef ROTOR_CONTROL_HPP
#define ROTOR_CONTROL_HPP

#define MOTOR1_PIN 20
#define AMP_OFFSET 0.0f //  amplitude offset to overcome static friction of hinge
#define SQ(x) ((x)*(x))

#include <Arduino.h>

namespace control::rotor
{
    inline TaskHandle_t rotorTaskHandle = NULL;

    void initRotor();
    void rotorControlTask(void *pvParameters);
    void sendToDshot(float throttle_fraction);
}

#endif // ROTOR_CONTROL_HPP