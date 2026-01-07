#include "mode_manager.hpp"

namespace flight {

    ModeManager::ModeManager() {} 

    void ModeManager::init() {
        xTaskCreate(modeManagerTaskEntry, "Mode Manager Task", 4096, this, 3, nullptr);
        Serial.println("[ModeManager] Mode Manager task started.");
    }
    
    void ModeManager::retrieveMessages() {
        vehicle_state_sub_.pull_if_new(vehicle_state_msg_);
        rc_command_sub_.pull_if_new(rc_command_msg_);
        rc_attitude_setpoint_.x() = rc_command_msg_.roll;
        rc_attitude_setpoint_.y() = rc_command_msg_.pitch;
        rc_attitude_setpoint_.z() = rc_command_msg_.yaw;
    }
    
    void ModeManager::modeManagerTask(void *pvParameters) {
        TickType_t last_wake_time = xTaskGetTickCount();        

        while (true) {

            // switch (curFlightMode_)
            // {
            // // only stabilized mode implemented for now
            // case FlightMode::STABILIZED:
                
            //     break;
            
            // default:
            //     break;
            // }

            vehicle_mode_msg_.timestamp = micros();
            vehicle_mode_msg_.flight_mode = curFlightMode_;
            vehicle_mode_pub_.push(vehicle_mode_msg_);
            
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(dt_ms_)); // 100 Hz
        }
    }

}