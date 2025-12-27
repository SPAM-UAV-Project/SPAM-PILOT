#include <Arduino.h>
#include "sensors/imu/imu.hpp"

void setup() 
{
    Serial.begin(921600); 
    sensors::imu::initIMU();  
}

void loop() {}
