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

    SystemState cur_state_ = SystemState::INITIALIZING;
    FlightMode cur_mode_ = FlightMode::STABILIZED;
    uint32_t dt = 20; // ms - 50 hz
};

};

#endif // STATE_MANAGER_HPP