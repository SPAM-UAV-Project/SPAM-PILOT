#ifndef SWITCH_STATE_SRV_HPP
#define SWITCH_STATE_SRV_HPP

#include "srv_broker.hpp"
#include "msgs/VehicleStateMsg.hpp"

/**
 * @param new_state The new system state to switch to
 * @param success Whether the state switch was successful
 */

namespace srv {

struct SwitchState {
    struct Request {
        SystemState new_state;
    };

    struct Response {
        bool success;
    };
};

} // namespace srv


#endif // SWITCH_STATE_SRV_HPP