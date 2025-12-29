#include <Arduino.h>
#include "sensors/imu/imu.hpp"
#include "flight/state_manager.hpp"


void setup() 
{
    Serial.begin(921600); 
    
    flight::StateManager state_manager;
    state_manager.init();
}

void loop() {}
