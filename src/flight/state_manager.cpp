#include "state_manager.hpp"
#include "sensors/imu/imu.hpp"
#include "msgs/ImuMagMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "gnc/control_allocator/rotor_control.hpp"

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

    // subscribers
    Topic<ImuMagMsg>::Subscriber imu_mag_sub;
    Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub;
    Topic<EkfStatesMsg>::Subscriber ekf_states_sub;
    ImuMagMsg mag_data;
    ImuHighRateMsg imu_highrate_data;
    EkfStatesMsg ekf_states_data;

    // advertise services
    Service<srv::SwitchState>::Server state_switcher_srv;
    state_switcher_srv.advertise<StateManager, &StateManager::onSwitchState>(this);

    while (true) {

        switch (cur_state_)
        {
        case SystemState::INITIALIZING:
            sensors::imu::initIMU();
            state_estimator_.init();
            // control::rotor::initRotor();
            // start mavlink, radio here
            switchState(SystemState::MOTORS_DISABLED);
            break;
        case SystemState::MOTORS_DISABLED:
            // wait for arming command to initialize any of the control loops for safety
            break;
        case SystemState::ARMED:
            // might just switch to IN_FLIGHT directly , nothing much to do here/ maybe final checks?
            switchState(SystemState::IN_FLIGHT);
            break;
        case SystemState::IN_FLIGHT:
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