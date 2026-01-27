/**
 * @file mavlink_comms.hpp
 * @brief MAVLink communication manager
 */

#ifndef MAVLINK_COMMS_HPP
#define MAVLINK_COMMS_HPP

#include <Arduino.h>
#include "transport/transport_base.hpp"
#include "common/mavlink.h"
#include "msgs/VehicleStateMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/RateSetpointMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"
#include "srvs/SwitchStateSrv.hpp"

namespace cdh::mavlink {

constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
constexpr uint8_t MAVLINK_COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;
constexpr size_t MAX_TRANSPORTS = 2;

/**
 * @brief Main MAVLink communication class
 * Implementation split across mavlink_comms.cpp, mavlink_pubs.cpp, mavlink_handlers.cpp
 */
class MavlinkComms {
public:
    MavlinkComms();
    ~MavlinkComms() = default;

    void init();
    bool registerTransport(TransportBase* transport);
    void sendMessage(const mavlink_message_t* msg);
    void sendBuffer(const uint8_t* buffer, size_t len);
    void log(uint8_t severity, const char* fmt, ...);

    uint8_t getSystemId() const { return sys_id_; }
    uint8_t getComponentId() const { return comp_id_; }
    bool isGcsConnected() const;

    static void rxTaskEntry(void* instance) { static_cast<MavlinkComms*>(instance)->rxTask(); }
    static void txTaskEntry(void* instance) { static_cast<MavlinkComms*>(instance)->txTask(); }

    TaskHandle_t rxTaskHandle = nullptr;
    TaskHandle_t txTaskHandle = nullptr;

private:
    // Core (mavlink_comms.cpp)
    void rxTask();
    void processIncoming();
    void dispatchMessage(const mavlink_message_t* msg);

    // Publishers (mavlink_pubs.cpp)
    void txTask();
    void publishHeartbeat();
    void publishAttitudeQuaternion();
    void publishSysStatus();
    void publishLocalPosition();
    void publishRcChannels();
    void publishImuHighRate();
    void publishAttControlSetpoints();
    void publishForceSetpoint();

    // Handlers (mavlink_handlers.cpp)
    void handleHeartbeat(const mavlink_message_t* msg);
    void handleCommandLong(const mavlink_message_t* msg);
    void handleSetMode(const mavlink_message_t* msg);
    void handleRequestDataStream(const mavlink_message_t* msg);

    // Helpers
    uint8_t systemStateToMavState(SystemState state);
    uint32_t flightModeToCustomMode(FlightMode mode);

    // Configuration
    uint8_t sys_id_ = MAVLINK_SYSTEM_ID;
    uint8_t comp_id_ = MAVLINK_COMPONENT_ID;

    // Transports
    TransportBase* transports_[MAX_TRANSPORTS] = {nullptr};
    size_t transport_count_ = 0;

    // Parser state
    mavlink_status_t rx_status_[MAX_TRANSPORTS];
    mavlink_message_t rx_msg_;
    static constexpr size_t RX_BUFFER_SIZE = 280;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];

    // GCS tracking
    uint32_t last_gcs_heartbeat_ms_ = 0;
    bool gcs_connected_ = false;

    // Internal data subscribers
    Topic<VehicleStateMsg>::Subscriber vehicle_state_sub_;
    Topic<EkfStatesMsg>::Subscriber ekf_states_sub_;
    Topic<ImuHighRateMsg>::Subscriber imu_high_rate_sub_;
    Topic<RcCommandMsg>::Subscriber rc_command_sub_;
    Topic<AttitudeSetpointMsg>::Subscriber attitude_setpoint_sub_;
    Topic<RateSetpointMsg>::Subscriber rate_setpoint_sub_;
    Topic<ThrustSetpointMsg>::Subscriber thrust_setpoint_sub_;
    VehicleStateMsg vehicle_state_;
    EkfStatesMsg ekf_states_;
    ImuHighRateMsg imu_high_rate_;
    RcCommandMsg rc_command_;
    AttitudeSetpointMsg attitude_setpoint_;
    RateSetpointMsg rate_setpoint_;
    ThrustSetpointMsg thrust_setpoint_;

};

extern MavlinkComms* g_mavlink_instance;

} // namespace cdh::mavlink

#define MAV_LOG_DEBUG(fmt, ...) do { if (cdh::mavlink::g_mavlink_instance) cdh::mavlink::g_mavlink_instance->log(MAV_SEVERITY_DEBUG, fmt, ##__VA_ARGS__); } while(0)
#define MAV_LOG_INFO(fmt, ...)  do { if (cdh::mavlink::g_mavlink_instance) cdh::mavlink::g_mavlink_instance->log(MAV_SEVERITY_INFO, fmt, ##__VA_ARGS__); } while(0)
#define MAV_LOG_WARN(fmt, ...)  do { if (cdh::mavlink::g_mavlink_instance) cdh::mavlink::g_mavlink_instance->log(MAV_SEVERITY_WARNING, fmt, ##__VA_ARGS__); } while(0)
#define MAV_LOG_ERROR(fmt, ...) do { if (cdh::mavlink::g_mavlink_instance) cdh::mavlink::g_mavlink_instance->log(MAV_SEVERITY_ERROR, fmt, ##__VA_ARGS__); } while(0)
#define MAV_LOG(fmt, ...) MAV_LOG_INFO(fmt, ##__VA_ARGS__)

#endif // MAVLINK_COMMS_HPP
