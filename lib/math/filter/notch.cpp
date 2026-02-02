#include "notch.hpp"
#include <math.h>

NotchFilt::NotchFilt()
    : b0n_(0.0f), b1n_(0.0f), b2n_(0.0f),
      a1n_(0.0f), a2n_(0.0f),
      x_1{0.0f, 0.0f, 0.0f},
      y_1{0.0f, 0.0f, 0.0f},
      x_2{0.0f, 0.0f, 0.0f},
      y_2{0.0f, 0.0f, 0.0f}
{}

void NotchFilt::setup(double notch_freq, double bandwidth, double sample_freq)
{
    if (bandwidth <= 0.0 || sample_freq <= 0.0) {
        return; 
    }

    double Q = notch_freq / bandwidth;
    double omega = 2.0 * M_PI * notch_freq / sample_freq;
    double alpha = sin(omega) / (2.0 * Q);

    double b0 = 1.0;
    double b1 = -2.0 * cos(omega);
    double b2 = 1.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * cos(omega);
    double a2 = 1.0 - alpha;

    // normalize
    b0n_ = (float)b0 / a0;
    b1n_ = (float)b1 / a0;
    b2n_ = (float)b2 / a0;
    a1n_ = (float)a1 / a0;
    a2n_ = (float)a2 / a0;   
}

void NotchFilt::apply3d(const float input[3], float output[3])
{
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    for (int i = 0; i < 3; ++i) {
        output[i] = b0n_*input[i] + b1n_*x_1[i] + b2n_*x_2[i] - a1n_*y_1[i] - a2n_*y_2[i];
        // update states
        x_2[i] = x_1[i];
        x_1[i] = input[i];
        y_2[i] = y_1[i];
        y_1[i] = output[i];
    }
}