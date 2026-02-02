#include "state_manager.hpp"

namespace flight {

StateManager::StateManager() {}

void StateManager::init() {
    xTaskCreate(stateManagerTaskEntry, "StateManagerTask", 8192, this, 2, nullptr);
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

void StateManager::processRcCommands()
{
    if (cur_state_ == SystemState::INITIALIZING) {
        return; // ignore rc commands during init
    }
    // pull rc command and check for mode switches
    rc_command_sub_.pull_if_new(rc_command_sub_msg_);

    if (rc_command_sub_msg_.emergency_stop && (cur_state_ != SystemState::FAILSAFE)) {
        switchState(SystemState::FAILSAFE);
        return;
    } else if (!rc_command_sub_msg_.emergency_stop && (cur_state_ == SystemState::FAILSAFE)) {
        switchState(SystemState::DISARMED);
        armable_ = false; // require switch low again to re-arm
        return;
    }

    bool arm_requested = rc_command_sub_msg_.arm_switch;

    // simple arming logic, no pre-arm checks yet
    if (cur_state_ != SystemState::FAILSAFE && armable_) {
        if (arm_requested && (cur_state_ == SystemState::DISARMED)) {
            switchState(SystemState::ARMED);
            return;
        } 
        // note; we should not disarm if we are in the air but for this mvp we will keep it
        else if (!arm_requested && 
                (cur_state_ == SystemState::ARMED || cur_state_ == SystemState::ARMED_FLYING)) {
            switchState(SystemState::DISARMED);
            return;    
        }
    }

    // if switch is low at least once after boot, we can allow arming (to prevent startup arming) this is not working right now, probably will implement if low for at least 1 second
    if (!arm_requested) {
        armable_ = true;
    }
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
            // delay(200);  // Allow I2C bus 1 to stabilize
            sensors::encoder::initEncoder();
            // delay(200);  // Allow I2C bus 0 to stabilize
            state_estimator_.init();
            radio_controller_.init();
            if (!sd_logger_.init()) {
                Serial.println("[StateManager] Warning: SD Logger init failed!");
            }

            // initialize controllers
            att_control_thread_.init();
            rate_control_thread_.init();
            control_allocator_.initRotor();
#ifdef MAVLINK_ENABLED
            // initialize mavlink
            //mavlink_comms_.registerTransport(&usb_transport_);
#ifdef WIFI_TRANSPORT
            mavlink_comms_.registerTransport(&wifi_transport_);
#endif // WIFI_TRANSPORT
            mavlink_comms_.init();
#endif // MAVLINK_ENABLED
            switchState(SystemState::DISARMED);
            break;
        case SystemState::DISARMED:
            // wait for arming command to initialize any of the control loops for safety
            sd_logger_.stopSession();
            break;
        case SystemState::ARMED:
            // might just switch to ARMED_FLYING directly , nothing much to do here/ maybe final checks?
            switchState(SystemState::ARMED_FLYING);
            sd_logger_.startSession(); // start logger
            break;
        case SystemState::ARMED_FLYING:
            // only have stabilized mode for now (hold attittude to level if no pilot input)
            // land detector
            break;
        case SystemState::FAILSAFE:
            sd_logger_.stopSession();
            break;
        default:
            break;
        }

        processRcCommands();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20)); // 50 Hz
    }
}

}