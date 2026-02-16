#ifndef INDI_CONTROLLER_HPP
#define INDI_CONTROLLER_HPP

#include "ArduinoEigen/Eigen/Dense"

namespace gnc
{
    class IndiController
    {
    public:
        IndiController(Eigen::Matrix3f G_effectiveness);
        ~IndiController() = default;

        Eigen::Vector3f runDeltaU(Eigen::Vector3f virtual_accel, Eigen::Vector3f ang_accel_meas);

    private:
        Eigen::Matrix3f G_effectiveness_;
        Eigen::Matrix3f G_effectiveness_inv_;
    };
    

} // namespace gnc


#endif // INDI_CONTROLLER_HPP