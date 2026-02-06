#include "integrator.hpp"

// Integrate Vector3f over dt and return integrated value on reset call.

Integrator::Integrator()
    : sum_(Eigen::Vector3f::Zero()),
      prev_value_(Eigen::Vector3f::Zero()),
      initialized_(false),
      total_dt_us_(0)
{
}

void Integrator::integrate3d(const Eigen::Vector3f& value, uint32_t dt_us) // dt in microseconds
{
    if (!initialized_) {
        prev_value_ = value;
        initialized_ = true;
    }

    // trapezoidal rule
    double dt_s = dt_us * 1e-6; // convert to seconds
    sum_ += (0.5 * (value + prev_value_) * dt_s).cast<float>();
    prev_value_ = value;

    total_dt_us_ += dt_us;
}

void Integrator::getAndReset3d(Eigen::Vector3f& output, float &int_time)
{
    output = sum_;
    int_time = total_dt_us_ * 1e-6; // convert to seconds
    
    sum_ = Eigen::Vector3f::Zero();
    prev_value_ = Eigen::Vector3f::Zero();
    initialized_ = false;
    total_dt_us_ = 0;
}
