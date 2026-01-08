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
        imu_high_rate_sub_.pull_if_new(imu_high_rate_);
        rc_command_sub_.pull_if_new(rc_command_);

        if (tick % 50 == 0) publishHeartbeat();      // 1 Hz
        publishAttitudeQuaternion();                  // 50 Hz
        if (tick % 50 == 0) publishSysStatus();      // 1 Hz
        if (tick % 5 == 0) publishLocalPosition();   // 10 Hz
        if (tick % 10 == 0) publishRcChannels();     // 5 Hz
        publishImuHighRate();                         // 50 Hz

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
        ekf_states_.attitude.w(),  // w (MAVLink expects w first)
        ekf_states_.attitude.x(),  // x
        ekf_states_.attitude.y(),  // y
        ekf_states_.attitude.z()   // z
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

void MavlinkComms::publishRcChannels() {
    mavlink_message_t msg;
    // rc channels is -1 to 1 and 0 to 1, need to convert to 1000-2000
    uint16_t chan[6];
    chan[0] = static_cast<uint16_t>(rc_command_.roll * 500.0f + 1500.0f);
    chan[1] = static_cast<uint16_t>(rc_command_.pitch * 500.0f + 1500.0f);
    chan[2] = static_cast<uint16_t>(rc_command_.yaw * 500.0f + 1500.0f);
    chan[3] = static_cast<uint16_t>(rc_command_.throttle * 1000.0f + 1000.0f);
    chan[4] = rc_command_.arm_switch ? 2000 : 1000;
    chan[5] = rc_command_.emergency_stop ? 1000 : 2000;
    
    mavlink_msg_rc_channels_pack(sys_id_, comp_id_, &msg,
        millis(),
        0,          // port
        chan[0], chan[1], chan[2], chan[3], chan[4], chan[5],
        0,0,0,0,0,0,0,0,0,0,0,0,  // chans 7-18 not used
        0);         // rssi
    sendMessage(&msg);
}

void MavlinkComms::publishImuHighRate() {
    mavlink_message_t msg;
    mavlink_msg_raw_imu_pack(sys_id_, comp_id_, &msg,
        imu_high_rate_.timestamp,
        static_cast<int16_t>(imu_high_rate_.accel(0) * 1000.0f),  // xacc
        static_cast<int16_t>(imu_high_rate_.accel(1) * 1000.0f),  // yacc
        static_cast<int16_t>(imu_high_rate_.accel(2) * 1000.0f),  // zacc
        static_cast<int16_t>(imu_high_rate_.gyro(0) * 1000.0f),   // xgyro
        static_cast<int16_t>(imu_high_rate_.gyro(1) * 1000.0f),   // ygyro
        static_cast<int16_t>(imu_high_rate_.gyro(2) * 1000.0f),   // zgyro
        0, 0, 0,            // mag (not used)
        0,                  // id
        0);                 // temperature (not used)
    sendMessage(&msg);
}

} // namespace cdh::mavlink
