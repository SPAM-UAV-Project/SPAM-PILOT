#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "msgs/VehicleStateMsg.hpp"
#include "gnc/state_estimation/state_estimator.hpp"
#include "srvs/SwitchStateSrv.hpp"
#include "cdh/mavlink/mavlink_comms.hpp"
#include "cdh/mavlink/transport/usb_transport.hpp"
#include "cdh/mavlink/transport/wifi_transport.hpp"

namespace flight {

class StateManager
{
public:
    StateManager();
    ~StateManager() = default;

    void init();

    static void stateManagerTaskEntry(void* instance) {
        static_cast<StateManager*>(instance)->stateManagerTask();
    }

private:
    void stateManagerTask();
    void switchState(SystemState new_state);
    bool onSwitchState(const srv::SwitchState::Request& req, srv::SwitchState::Response& res);

    SystemState cur_state_ = SystemState::INITIALIZING;
    FlightMode cur_mode_ = FlightMode::STABILIZED;

    // classes
    gnc::StateEstimator state_estimator_;

    // mavlink
    cdh::mavlink::MavlinkComms mavlink_comms_;
    cdh::mavlink::UsbTransport usb_transport_;
    cdh::mavlink::WifiTransport wifi_transport_;

};
}

#endif // STATE_MANAGER_HPP