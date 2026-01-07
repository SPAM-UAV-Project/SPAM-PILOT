#ifndef VEHICLE_FLIGHT_MODE_MSG_HPP
#define VEHICLE_FLIGHT_MODE_MSG_HPP

#include "msg_broker.hpp"

enum class FlightMode
{
    STABILIZED,
    ALT_HOLD,
    POS_HOLD
};

/**
 * @param timestamp Timestamp in microseconds
 * @param flight_mode Current flight mode
 */
struct VehicleFlightModeMsg
{
    uint64_t timestamp = 0;
    FlightMode flight_mode = FlightMode::STABILIZED;
};

#endif // VEHICLE_FLIGHT_MODE_MSG_HPP