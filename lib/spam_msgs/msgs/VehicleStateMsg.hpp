#ifndef VEHICLE_STATE_MSG_HPP
#define VEHICLE_STATE_MSG_HPP

#include "msg_broker.hpp"

enum class SystemState
{
    INITIALIZING,
    DISARMED,
    ARMED,
    ARMED_FLYING,
    FAILSAFE,
    CALIBRATION
};

/**
 * @param timestamp Timestamp in microseconds
 * @param system_state Current system state
 */
struct VehicleStateMsg
{
    uint64_t timestamp = 0;
    SystemState system_state = SystemState::INITIALIZING;
};

#endif // VEHICLE_STATE_MSG_HPP