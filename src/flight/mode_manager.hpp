#ifndef MODE_MANAGER_HPP
#define MODE_MANAGER_HPP

#include <Arduino.h>
#include "msgs/VehicleStateMsg.hpp"
#include "msgs/VehicleFlightModeMsg.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/ThrustSetpointMsg.hpp"

namespace flight {

    class ModeManager
    {
    public:
        ModeManager();
        ~ModeManager() = default;

        void init();

    private:
        
        void retrieveMessages();
        void modeManagerTask(void *pvParameters);
        static void modeManagerTaskEntry(void *instance) {
            static_cast<ModeManager*>(instance)->modeManagerTask(instance);
        }

        Topic<VehicleStateMsg>::Subscriber vehicle_state_sub_;  
        Topic<RcCommandMsg>::Subscriber rc_command_sub_;
        Topic<VehicleFlightModeMsg>::Publisher vehicle_mode_pub_;
        Topic<AttitudeSetpointMsg>::Publisher att_setpoint_pub_;
        Topic<ThrustSetpointMsg>::Publisher thrust_setpoint_pub_;

        VehicleStateMsg vehicle_state_msg_;
        RcCommandMsg rc_command_msg_;
        VehicleFlightModeMsg vehicle_mode_msg_;
        AttitudeSetpointMsg att_setpoint_msg_; 
        ThrustSetpointMsg thrust_setpoint_msg_;

        FlightMode curFlightMode_ = FlightMode::STABILIZED;
        Eigen::Vector3f rc_attitude_setpoint_ = Eigen::Vector3f::Zero(); // roll, pitch, yaw from RC

        float dt_ms_ = 10.0f; // mode manager loop time in ms
    };

}

#endif // MODE_MANAGER_HPP