#include "state_manager.hpp"

namespace flight {

StateManager::StateManager() {}

void StateManager::init() {
    xTaskCreate(stateManagerTaskEntry, "StateManagerTask", 8192, this, 3, nullptr);
    Serial.println("[StateManager] State Manager task started.");
}

void StateManager::switchState(SystemState new_state) {
    cur_state_ = new_state;
    Serial.printf("[StateManager] Switched to state: %d\n", static_cast<int>(cur_state_));
}

bool StateManager::onSwitchState(const srv::SwitchState::Request& req, srv::SwitchState::Response& res) {
    switchState(req.new_state);
    res.success = true;
    return true;
}

void StateManager::stateManagerTask() {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // advertise services
    state_switcher_srv_.advertise<StateManager, &StateManager::onSwitchState>(this);

    while (true) {

        // publish state
        vehicle_state_msg_.timestamp = micros();
        vehicle_state_msg_.system_state = cur_state_;
        vehicle_state_msg_.flight_mode = cur_flight_mode_;
        vehicle_state_pub_.push(vehicle_state_msg_);

        switch (cur_state_)
        {
        case SystemState::INITIALIZING:
            sensors::imu::initIMU();
            state_estimator_.init();
            radio_controller_.init();

            // initialize controllers
            att_control_thread_.init();
            rate_control_thread_.init();
            // control::rotor::initRotor();

            // initialize mavlink
            mavlink_comms_.registerTransport(&usb_transport_);
            // mavlink_comms_.registerTransport(&wifi_transport_);
            mavlink_comms_.init();
            
            switchState(SystemState::DISARMED);
            break;
        case SystemState::DISARMED:
            // wait for arming command to initialize any of the control loops for safety

            break;
        case SystemState::ARMED:
            // might just switch to ARMED_FLYING directly , nothing much to do here/ maybe final checks?
            switchState(SystemState::ARMED_FLYING);
            break;
        case SystemState::ARMED_FLYING:
            // only have stabilized mode for now (hold attittude to level if no pilot input)

            // land detector
            break;
        case SystemState::FAILSAFE:
            break;
        default:
            break;
        }


        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20)); // 50 Hz
    }
}

}