#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "msgs/VehicleStateMsg.hpp"
#include "gnc/state_estimation/state_estimator.hpp"

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

    SystemState cur_state_ = SystemState::INITIALIZING;
    FlightMode cur_mode_ = FlightMode::STABILIZED;

    // classes
    gnc::StateEstimator state_estimator_;

};
}

#endif // STATE_MANAGER_HPP