#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "msgs/VehicleStateMsg.hpp"
#include "sensors/imu/imu.hpp"
#include "sensors/encoder/encoder.hpp"
#include "gnc/state_estimation/state_estimator.hpp"
#include "gnc/att_control_thread/att_control_thread.hpp"
#include "gnc/rate_control_thread/rate_control_thread.hpp"
#include "cdh/radio_controller/rc.hpp"
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

private:
    void stateManagerTask();
    static void stateManagerTaskEntry(void* instance) {
        static_cast<StateManager*>(instance)->stateManagerTask();
    }
    void switchState(SystemState new_state);
    bool onSwitchState(const srv::SwitchState::Request& req, srv::SwitchState::Response& res);

    // classes
    gnc::StateEstimator state_estimator_;
    cdh::RadioController radio_controller_;
    gnc::AttControlThread att_control_thread_;
    gnc::RateControlThread rate_control_thread_;


    SystemState cur_state_ = SystemState::INITIALIZING;
    FlightMode cur_flight_mode_ = FlightMode::STABILIZED;

    // subscribers
    Topic<RcCommandMsg>::Subscriber rc_command_sub;

    // publishers and srvs
    Topic<VehicleStateMsg>::Publisher vehicle_state_pub_;
    Service<srv::SwitchState>::Server state_switcher_srv_;
    VehicleStateMsg vehicle_state_msg_;

    // mavlink
    cdh::mavlink::MavlinkComms mavlink_comms_;
    cdh::mavlink::UsbTransport usb_transport_;
    cdh::mavlink::WifiTransport wifi_transport_;

};
}

#endif // STATE_MANAGER_HPP