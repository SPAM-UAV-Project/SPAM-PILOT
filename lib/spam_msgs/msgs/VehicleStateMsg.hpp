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

enum class FlightMode
{
    STABILIZED,
    ALT_HOLD,
    POS_HOLD
};

/**
 * @param timestamp Timestamp in microseconds
 * @param system_state Current system state
 * @param flight_mode Current flight mode
 */
struct VehicleStateMsg
{
    uint64_t timestamp = 0;
    SystemState system_state = SystemState::INITIALIZING;
    FlightMode flight_mode = FlightMode::STABILIZED;
};

#endif // VEHICLE_STATE_MSG_HPP