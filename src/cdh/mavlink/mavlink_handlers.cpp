/**
 * @file mavlink_handlers.cpp
 * @brief MavlinkComms handler methods - incoming message processing
 */

#include "mavlink_comms.hpp"

namespace cdh::mavlink {

void MavlinkComms::handleHeartbeat(const mavlink_message_t* msg) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);

    if (hb.type == MAV_TYPE_GCS) {
        last_gcs_heartbeat_ms_ = millis();
        if (!gcs_connected_) {
            gcs_connected_ = true;
            log(MAV_SEVERITY_INFO, "GCS connected");
        }
    }
}

void MavlinkComms::handleCommandLong(const mavlink_message_t* msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            bool arm = (cmd.param1 > 0.5f);
            
            Service<srv::SwitchState>::Client client;
            srv::SwitchState::Request req;
            srv::SwitchState::Response res;
            
            req.new_state = arm ? SystemState::ARMED : SystemState::MOTORS_DISABLED;
            log(MAV_SEVERITY_INFO, arm ? "Arm cmd" : "Disarm cmd");
            
            result = client.call(req, res) && res.success 
                ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
            break;
        }

        case MAV_CMD_DO_SET_MODE:
            log(MAV_SEVERITY_INFO, "Mode: %d", (int)cmd.param2);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_REQUEST_MESSAGE:
            result = MAV_RESULT_ACCEPTED;
            break;
    }

    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(sys_id_, comp_id_, &ack,
        cmd.command, result, 0, 0, msg->sysid, msg->compid);
    sendMessage(&ack);
}

void MavlinkComms::handleSetMode(const mavlink_message_t* msg) {
    mavlink_set_mode_t mode;
    mavlink_msg_set_mode_decode(msg, &mode);

    if (mode.target_system != sys_id_) return;

    Service<srv::SwitchState>::Client client;
    srv::SwitchState::Request req;
    srv::SwitchState::Response res;

    req.new_state = (mode.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        ? SystemState::ARMED : SystemState::MOTORS_DISABLED;
    client.call(req, res);
}

void MavlinkComms::handleRequestDataStream(const mavlink_message_t* msg) {
    // Fixed rates in txTask
}

} // namespace cdh::mavlink
