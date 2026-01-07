#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "msgs/VehicleStateMsg.hpp"
#include "gnc/state_estimation/state_estimator.hpp"
#include "cdh/radio_controller/rc.hpp"
#include "srvs/SwitchStateSrv.hpp"

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
    cdh::RadioController radio_controller_;

};
}

#endif // STATE_MANAGER_HPP