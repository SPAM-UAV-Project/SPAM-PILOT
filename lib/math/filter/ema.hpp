#ifndef MATH_FILTER_EMA_HPP
#define MATH_FILTER_EMA_HPP

#include <ArduinoEigen/Eigen/Dense>

class EmaLowPassFilter {
public:
    EmaLowPassFilter();

    void setTimeConst(float time_const, float dt);

    float getAlpha() const {
        return alpha_;
    }

    Eigen::Vector3f apply3d(const Eigen::Vector3f& input);

    void reset() {
        initialized_ = false;
    }

private:
    float alpha_;
    Eigen::Vector3f output_;
    bool initialized_;
};

#endif // MATH_FILTER_EMA_HPP