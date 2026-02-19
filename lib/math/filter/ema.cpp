#include "ema.hpp"

EmaLowPassFilter::EmaLowPassFilter()
    : alpha_(0.0f), output_(0.0f, 0.0f, 0.0f), initialized_(false) {}

void EmaLowPassFilter::setTimeConst(float time_const, float dt) {
    if (time_const <= 0.0f) {
        alpha_ = 1.0f; // no filtering
    } else {
        alpha_ = dt / (time_const + dt);
    }
}

Eigen::Vector3f EmaLowPassFilter::apply3d(const Eigen::Vector3f& input) {
    if (!initialized_) {
        output_ = input;
        initialized_ = true;
    } else {
        output_ = output_ + alpha_ * (input - output_);
    }
    return output_;
}