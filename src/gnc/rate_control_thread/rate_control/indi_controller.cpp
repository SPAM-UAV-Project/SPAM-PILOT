#include "gnc/rate_control_thread/rate_control/indi_controller.hpp"

namespace gnc
{
    IndiController::IndiController(Eigen::Matrix3f G_effectiveness)
        : G_effectiveness_(G_effectiveness)
    {
        G_effectiveness_inv_ = G_effectiveness_.inverse();
    }

    Eigen::Vector3f IndiController::runDeltaU(Eigen::Vector3f virtual_accel, Eigen::Vector3f ang_accel_meas)
    {
        Eigen::Vector3f ang_accel_error = virtual_accel - ang_accel_meas;
        return Eigen::Vector3f(G_effectiveness_inv_(0, 0) * ang_accel_error(0),
                               G_effectiveness_inv_(1, 1) * ang_accel_error(1),
                               G_effectiveness_inv_(2, 2) * ang_accel_error(2));
    }
    
} // namespace gnc
