#include "butter_lp.hpp"
#include <math.h>

ButterLowPassFilt::ButterLowPassFilt(float cutoff_freq, float sample_freq)
    : cutoff_freq_(cutoff_freq), sample_freq_(sample_freq)
{
    float K = tanf(M_PI * cutoff_freq / sample_freq);
    float norm = 1.0f / (1.0f + sqrtf(2) * K + K * K);
    b0_ = K*K * norm;
    b1_ = 2.0f * b0_;
    b2_ = b0_;
    a1_ = 2.0f * (K*K - 1) * norm;
    a2_ = (1.0f - sqrtf(2) * K + K*K) * norm;
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
