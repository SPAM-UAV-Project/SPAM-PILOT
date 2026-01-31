/**
 * @file mavlink_comms.cpp
 * @brief MAVLink core - transport management, parsing, dispatching
 */

#include "mavlink_comms.hpp"
#include <cstdarg>

namespace cdh::mavlink {

MavlinkComms* g_mavlink_instance = nullptr;

MavlinkComms::MavlinkComms() {
    memset(rx_status_, 0, sizeof(rx_status_));
}

void MavlinkComms::init() {
    g_mavlink_instance = this;

    for (size_t i = 0; i < transport_count_; i++) {
        if (transports_[i]) transports_[i]->begin();
    }

    xTaskCreate(rxTaskEntry, "MavRxTask", 4096, this, 1, &rxTaskHandle);
    xTaskCreate(txTaskEntry, "MavTxTask", 8192, this, 1, &txTaskHandle);
    
    Serial.println("[MavlinkComms] Tasks started");
}

bool MavlinkComms::registerTransport(TransportBase* transport) {
    if (transport_count_ >= MAX_TRANSPORTS) return false;
    transports_[transport_count_++] = transport;
    return true;
}

void MavlinkComms::sendMessage(const mavlink_message_t* msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    size_t len = mavlink_msg_to_send_buffer(buffer, msg);
    sendBuffer(buffer, len);
}

void MavlinkComms::sendBuffer(const uint8_t* buffer, size_t len) {
    for (size_t i = 0; i < transport_count_; i++) {
        if (transports_[i] && transports_[i]->isConnected()) {
            transports_[i]->send(buffer, len);
        }
    }
}

void MavlinkComms::log(uint8_t severity, const char* fmt, ...) {
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(sys_id_, comp_id_, &msg, severity, text, 0, 0);
    sendMessage(&msg);
}

bool MavlinkComms::isGcsConnected() const {
    return gcs_connected_ && (millis() - last_gcs_heartbeat_ms_) < 5000;
}

void MavlinkComms::rxTask() {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        processIncoming();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

void MavlinkComms::processIncoming() {
    for (size_t t = 0; t < transport_count_; t++) {
        if (!transports_[t]) continue;
        size_t avail = transports_[t]->available();
        if (avail == 0) continue;

        size_t to_read = (avail < RX_BUFFER_SIZE) ? avail : RX_BUFFER_SIZE;
        size_t bytes = transports_[t]->receive(rx_buffer_, to_read);

        for (size_t i = 0; i < bytes; i++) {
            if (mavlink_parse_char(t, rx_buffer_[i], &rx_msg_, &rx_status_[t])) {
                dispatchMessage(&rx_msg_);
            }
        }
    }
}

void MavlinkComms::dispatchMessage(const mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:           handleHeartbeat(msg); break;
        case MAVLINK_MSG_ID_COMMAND_LONG:        handleCommandLong(msg); break;
        case MAVLINK_MSG_ID_SET_MODE:            handleSetMode(msg); break;
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: handleRequestDataStream(msg); break;
    }
}

uint8_t MavlinkComms::systemStateToMavState(SystemState state) {
    switch (state) {
        case SystemState::INITIALIZING:    return MAV_STATE_BOOT;
        case SystemState::DISARMED: return MAV_STATE_STANDBY;
        case SystemState::ARMED:    return MAV_STATE_ACTIVE;
        case SystemState::ARMED_FLYING:       return MAV_STATE_ACTIVE;
        case SystemState::FAILSAFE:        return MAV_STATE_CRITICAL;
        case SystemState::CALIBRATION:     return MAV_STATE_CALIBRATING;
        default:                           return MAV_STATE_UNINIT;
    }
}

uint32_t MavlinkComms::flightModeToCustomMode(FlightMode mode) {
    return static_cast<uint32_t>(mode);
}

} // namespace cdh::mavlink
