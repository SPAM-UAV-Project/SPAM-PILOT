#include <Arduino.h>
#include "sensors/imu/imu.hpp"
#include "flight/state_manager.hpp"

flight::StateManager state_manager;

void setup() 
{
    Serial.begin(921600); 
    state_manager.init();
}

void loop() {

    // // Allocate a buffer (approx 40 bytes per task)
    // char statsBuffer[1024]; 
    
    // Serial.println("\nTask Name\tAbs Time\tTime %");
    // Serial.println("------------------------------------------");
    
    // // This function fills the buffer with a formatted table
    // vTaskGetRunTimeStats(statsBuffer);
    
    // Serial.println(statsBuffer);
    
    // // Print every 5 seconds
    // vTaskDelay(pdMS_TO_TICKS(5000));
}
