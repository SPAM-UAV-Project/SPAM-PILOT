#include "state_manager.hpp"

namespace flight {

StateManager::StateManager() {}

void StateManager::init() {
    xTaskCreate(stateManagerTaskEntry, "StateManagerTask", 4096, this, 2, nullptr);
    Serial.println("[StateManager] State Manager task started.");
}

void StateManager::stateManagerTask() {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {

        switch (cur_state_)
        {
        case SystemState::INITIALIZING:
            break;
        case SystemState::MOTORS_DISABLED:
            break;
        case SystemState::ARMED:
            break;
        case SystemState::IN_FLIGHT:
            break;
        case SystemState::FAILSAFE:
            break;
        default:
            break;
        }
    

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(dt));
    }
}

}