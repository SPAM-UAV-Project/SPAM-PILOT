#ifndef MATH_INTEGRATOR_INTEGRATOR_HPP
#define MATH_INTEGRATOR_INTEGRATOR_HPP

#include "ArduinoEigen/Eigen/Dense"

class Integrator {
public:
    Integrator();
    ~Integrator() = default;

    void integrate3d(const Eigen::Vector3f& value, uint32_t dt_us);
    void getAndReset3d(Eigen::Vector3f& output, float &int_time);
    bool isReady(uint32_t min_int_time_us) const {
        return total_dt_us_ >= min_int_time_us;
    }

private: 
    Eigen::Vector3f sum_;
    Eigen::Vector3f prev_value_;
    bool initialized_;
    uint32_t total_dt_us_;
};

#endif // MATH_INTEGRATOR_INTEGRATOR_HPP