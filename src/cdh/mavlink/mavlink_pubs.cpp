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
        attitude_setpoint_sub_.pull_if_new(attitude_setpoint_);
        rate_setpoint_sub_.pull_if_new(rate_setpoint_);
        thrust_setpoint_sub_.pull_if_new(thrust_setpoint_);
        encoder_sub_.pull_if_new(encoder_);
        motor_forces_sub_.pull_if_new(motor_forces_);

        if (tick % 50 == 0) { // 1 Hz
            publishHeartbeat(); 
            publishSysStatus();      
        } 
        if (tick % 5 == 0) {
            publishLocalPosition(); 
            publishEncoderAngle();   // 10 Hz
        }
        if (tick % 10 == 0) {
            publishRcChannels();     // 5 Hz
        }
        publishAttitudeQuaternion();                  // 50 Hz
        publishImuHighRate();                         // 50 Hz
        publishAttControlSetpoints();                // 50 Hz
        publishMotorForces(); // 50 Hz

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
    chan[2] = static_cast<uint16_t>(rc_command_.throttle * 1000.0f + 1000.0f);
    chan[3] = static_cast<uint16_t>(rc_command_.yaw * 500.0f + 1500.0f);
    chan[4] = rc_command_.arm_switch ? 2000 : 1000;
    chan[5] = rc_command_.emergency_stop ? 2000 : 1000;
    
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
    mavlink_msg_highres_imu_pack(sys_id_, comp_id_, &msg,
        imu_high_rate_.timestamp,
        imu_high_rate_.accel(0) - ekf_states_.accel_bias(0),  // xacc
        imu_high_rate_.accel(1) - ekf_states_.accel_bias(1),  // yacc
        imu_high_rate_.accel(2) - ekf_states_.accel_bias(2),  // zacc
        imu_high_rate_.gyro_filtered(0) - ekf_states_.gyro_bias(0),  // xgyro
        imu_high_rate_.gyro_filtered(1) - ekf_states_.gyro_bias(1),  // ygyro
        imu_high_rate_.gyro_filtered(2) - ekf_states_.gyro_bias(2),  // zgyro
        0, 0, 0,            // mag (not used)
        0,                  // id
        0,
        0, 0, 0, 0
    );                 // temperature (not used)
    sendMessage(&msg);
}

void MavlinkComms::publishAttControlSetpoints() {
    mavlink_message_t msg;
    float q[4] = {
        attitude_setpoint_.q_sp.w(),
        attitude_setpoint_.q_sp.x(),
        attitude_setpoint_.q_sp.y(),
        attitude_setpoint_.q_sp.z()
    };
    mavlink_msg_set_attitude_target_pack(sys_id_, comp_id_, &msg,
        millis(),
        0,  // target system
        0,  // target component
        0b00000000,  // type mask (not used)
        q,
        rate_setpoint_.setpoint(0),
        rate_setpoint_.setpoint(1),
        rate_setpoint_.setpoint(2),
        thrust_setpoint_.setpoint,
        nullptr);  // thrust body (not used);
    sendMessage(&msg);
}

void MavlinkComms::publishEncoderAngle() {
    mavlink_message_t msg;
    mavlink_msg_named_value_float_pack(sys_id_, comp_id_, &msg,
        millis(),
        "enc_angle",
        encoder_.angle_rad);
    sendMessage(&msg);
}

void MavlinkComms::publishMotorForces() {
    mavlink_message_t msg;
    float data[4] = {
        motor_forces_.setpoint(0),
        motor_forces_.setpoint(1),
        motor_forces_.setpoint(2),
        motor_forces_.setpoint(3)
    };
    mavlink_msg_debug_float_array_pack(sys_id_, comp_id_, &msg,
        micros(),        // time_usec (microseconds)
        "motor_f",  // name
        0,               // array_id
        data);           // float array
    sendMessage(&msg);
}

} // namespace cdh::mavlink
