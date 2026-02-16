#include "butter_lp.hpp"
#include <math.h>

ButterLowPassFilt::ButterLowPassFilt()
    : b0_(0.0f), b1_(0.0f), b2_(0.0f),
      a1_(0.0f), a2_(0.0f),
      x_1{0.0f, 0.0f, 0.0f},
      y_1{0.0f, 0.0f, 0.0f},
      x_2{0.0f, 0.0f, 0.0f},
      y_2{0.0f, 0.0f, 0.0f}
{}

void ButterLowPassFilt::setup(float cutoff_freq, float sample_freq)
{
    double K = tan(M_PI * cutoff_freq / sample_freq);
    double norm = 1.0 / (1.0 + sqrt(2) * K + K * K);
    double b0_d = K*K * norm;
    double b1_d = 2.0 * b0_d;
    double b2_d = b0_d;
    double a1_d = 2.0 * (K*K - 1) * norm;
    double a2_d = (1.0 - sqrt(2) * K + K*K) * norm;

    b0_ = (float)b0_d;
    b1_ = (float)b1_d;
    b2_ = (float)b2_d;
    a1_ = (float)a1_d;
    a2_ = (float)a2_d;
}

void ButterLowPassFilt::apply3d(const float input[3], float output[3])
{
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    for (int i = 0; i < 3; ++i) {
        output[i] = b0_*input[i] + b1_*x_1[i] + b2_*x_2[i] - a1_*y_1[i] - a2_*y_2[i];
        // update states
        x_2[i] = x_1[i];
        x_1[i] = input[i];
        y_2[i] = y_1[i];
        y_1[i] = output[i];
    }
}

void ButterLowPassFilt::apply1d(float input, float& output)
{
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    output = b0_*input + b1_*x_1[0] + b2_*x_2[0] - a1_*y_1[0] - a2_*y_2[0];
    // update states
    x_2[0] = x_1[0];
    x_1[0] = input;
    y_2[0] = y_1[0];
    y_1[0] = output;
}

void ButterLowPassFilt::reset()
{
    for (int i = 0; i < 3; ++i) {
        x_1[i] = 0.0f;
        x_2[i] = 0.0f;
        y_1[i] = 0.0f;
        y_2[i] = 0.0f;
    }
}