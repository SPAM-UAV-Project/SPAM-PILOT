#ifndef RC_COMMAND_MSG_HPP
#define RC_COMMAND_MSG_HPP

#include "msg_broker.hpp"
#include "msgs/VehicleStateMsg.hpp"

struct RcCommandMsg
{
    uint64_t timestamp = 0;
    float pitch = 0.0f; // -1 to 1
    float roll = 0.0f;  // -1 to 1
    float yaw = 0.0f;   // -1 to 1
    float throttle = 0.0f; // 0 to 1
    bool arm_switch = false;
    bool emergency_stop = true;
    FlightMode flight_mode = FlightMode::STABILIZED;
};

#endif // RC_COMMAND_MSG_HPP