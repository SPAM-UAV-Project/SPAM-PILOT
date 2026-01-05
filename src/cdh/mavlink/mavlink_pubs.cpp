/**
 * @file mavlink_pubs.cpp
 * @brief MavlinkComms publisher methods - telemetry TX
 */

#include "mavlink_comms.hpp"

namespace cdh::mavlink {

void MavlinkComms::txTask() {
    uint32_t tick = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vehicle_state_sub_.pull_if_new(vehicle_state_);
        ekf_states_sub_.pull_if_new(ekf_states_);

        if (tick % 50 == 0) publishHeartbeat();      // 1 Hz
        publishAttitudeQuaternion();                  // 50 Hz
        if (tick % 50 == 0) publishSysStatus();      // 1 Hz
        if (tick % 5 == 0) publishLocalPosition();   // 10 Hz

        tick++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

void MavlinkComms::publishHeartbeat() {
    mavlink_message_t msg;
    uint8_t base_mode = (systemStateToMavState(vehicle_state_.system_state) == MAV_STATE_ACTIVE)
        ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    
    mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, base_mode,
        flightModeToCustomMode(vehicle_state_.flight_mode),
        systemStateToMavState(vehicle_state_.system_state));
    sendMessage(&msg);
}

void MavlinkComms::publishAttitudeQuaternion() {
    // EkfStatesMsg uses (x, y, z, w) quaternion format
    float q[4] = {
        ekf_states_.attitude(3),  // w (MAVLink expects w first)
        ekf_states_.attitude(0),  // x
        ekf_states_.attitude(1),  // y
        ekf_states_.attitude(2)   // z
    };

    mavlink_message_t msg;
    mavlink_msg_attitude_quaternion_pack(sys_id_, comp_id_, &msg,
        millis(),
        q[0], q[1], q[2], q[3],  // quaternion (w, x, y, z)
        ekf_states_.gyro_bias(0),  // rollspeed
        ekf_states_.gyro_bias(1),  // pitchspeed
        ekf_states_.gyro_bias(2),  // yawspeed
        nullptr);  // repr_offset_q (not used)
    sendMessage(&msg);
}

void MavlinkComms::publishSysStatus() {
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(sys_id_, comp_id_, &msg,
        0, 0, 0, 0,      // sensors
        12000, -1, -1,   // battery (mV, cA, %)
        0, 0,            // drop_rate, errors_comm
        0, 0, 0, 0,      // errors_count
        0, 0, 0);        // ext sensors
    sendMessage(&msg);
}

void MavlinkComms::publishLocalPosition() {
    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(sys_id_, comp_id_, &msg,
        millis(),
        ekf_states_.position(0), ekf_states_.position(1), ekf_states_.position(2),
        ekf_states_.velocity(0), ekf_states_.velocity(1), ekf_states_.velocity(2));
    sendMessage(&msg);
}

} // namespace cdh::mavlink
