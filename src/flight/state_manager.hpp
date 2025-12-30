#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "msgs/VehicleStateMsg.hpp"

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
};

};

#endif // STATE_MANAGER_HPP