#include <Arduino.h>
#include "sensors/imu/imu.hpp"
#include "flight/state_manager.hpp"
#include "msgs/OpticalFlowMsg.hpp"

flight::StateManager state_manager;

// temp optiflow testing
Topic<OpticalFlowMsg>::Subscriber optiflow_sub;
OpticalFlowMsg optiflow_msg;

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

    // print optiflow
    // if (optiflow_sub.pull_if_new(optiflow_msg)) {
    //     Serial.printf("Optiflow - Distance: %.2f m, Vel X: %.2f m/s, Vel Y: %.2f m/s, Quality: %d\n", 
    //         optiflow_msg.distance, optiflow_msg.flow_vel_x, optiflow_msg.flow_vel_y, optiflow_msg.flow_quality);
    // }

    // delay(100);
}
