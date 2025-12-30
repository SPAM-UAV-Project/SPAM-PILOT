#include "state_manager.hpp"
#include "sensors/imu/imu.hpp"
#include "msgs/ImuMagMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"

namespace flight {

StateManager::StateManager() {}

void StateManager::init() {
    xTaskCreate(stateManagerTaskEntry, "StateManagerTask", 4096, this, 2, nullptr);
    Serial.println("[StateManager] State Manager task started.");
}

void StateManager::switchState(SystemState new_state) {
    cur_state_ = new_state;
    Serial.printf("[StateManager] Switched to state: %d\n", static_cast<int>(cur_state_));
}

void StateManager::stateManagerTask() {
    TickType_t last_wake_time = xTaskGetTickCount();

    // subscribers
    Topic<ImuMagMsg>::Subscriber imu_mag_sub;
    Topic<ImuHighRateMsg>::Subscriber imu_highrate_sub;
    ImuMagMsg mag_data;
    ImuHighRateMsg imu_highrate_data;

    while (true) {

        switch (cur_state_)
        {
        case SystemState::INITIALIZING:
            sensors::imu::initIMU();
            switchState(SystemState::MOTORS_DISABLED);
            break;
        case SystemState::MOTORS_DISABLED:
            // imu_highrate_sub.pull_if_new(imu_highrate_data);
            // if (imu_mag_sub.pull_if_new(mag_data)) {
            //     // print mag data and accel in x axis only
            //     Serial.printf("[IMU MAG] Mag: [%.2f, %.2f, %.2f] Accel X: %.2f\n",
            //                 mag_data.mag.x(), mag_data.mag.y(), mag_data.mag.z(),
            //                 imu_highrate_data.accel.x());
            // }

            
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
    

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20)); // 50 Hz
    }
}

}