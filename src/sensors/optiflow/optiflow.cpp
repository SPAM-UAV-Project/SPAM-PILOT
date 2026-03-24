/**
 * @file encoder.cpp
 * @brief Interfaces the AS5600 encoder to get angle measurements 
 */

#include "optiflow.hpp"
#include "mtf02p.hpp"
#include "pin_defs.hpp"
#include "msgs/OpticalFlowMsg.hpp"
#include "timing/task_timing.hpp"

namespace sensors::optiflow
{
    

    void initOptiflow()
    {
        // Create serial interface
        Serial1.setRxBufferSize(1024);
        Serial1.begin(115200, SERIAL_8N1, PIN_OPTIFLOW_RX, PIN_OPTIFLOW_TX); // rx 38, tx 39
        delay(200);
        while (Serial1.available() > 0) {
            (void)Serial1.read();
        }

        // start freertos tasks
        xTaskCreate(optiflowTask, "OptiflowTask", 4096, NULL, 3, &optiflowTaskHandle);
    
        Serial.println("[Optiflow]: Initialized");
    }

    void optiflowTask(void *pvParameters)
    {
        MICOLINK_PAYLOAD_RANGE_SENSOR_t msg = {};

        OpticalFlowMsg optiflow_msg;
        Topic<OpticalFlowMsg>::Publisher optiflow_pub;

        while(1){
            // read micolink
            while (Serial1.available() > 0) {
                uint8_t data = Serial1.read();
                const uint8_t has_new_data = micolink_decode(data, &msg);
                if (has_new_data) {
                    // publish data
                    optiflow_msg.timestamp = micros();
                    optiflow_msg.distance = msg.distance / 1000.0f; // convert mm to m
                    optiflow_msg.distance_precision = msg.precision;
                    optiflow_msg.distance_status = msg.dis_status;
                    optiflow_msg.flow_vel_x = msg.flow_vel_x * optiflow_msg.distance / 100.0f; // convert to m/s at distance reading
                    optiflow_msg.flow_vel_y = msg.flow_vel_y * optiflow_msg.distance / 100.0f;
                    optiflow_msg.flow_quality = msg.flow_quality;
                    optiflow_msg.flow_status = msg.flow_status;
                    optiflow_pub.push(optiflow_msg);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(2)); // yield to other tasks

            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 1250 == 0) {
            //     task_timer.printStats();
            //     Serial.println("Angular Velocity (rad/s): " + String(encoder_msg.angular_velocity_rad_s));
            // }
        }
    }    
}