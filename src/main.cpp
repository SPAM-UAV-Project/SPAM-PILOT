#include <Arduino.h>
#include "sensors/imu/imu.hpp"
#include "flight/state_manager.hpp"

flight::StateManager state_manager;

void setup() 
{
    Serial.begin(921600); 
    state_manager.init();
}

void loop() {}
