/***
 * Butterworth low pass filter implementation
 */

#ifndef MATH_FILTER_BUTTER_LP_HPP
#define MATH_FILTER_BUTTER_LP_HPP

class ButterLowPassFilt
{
public:
    ButterLowPassFilt(float cutoff_freq, float sample_freq);
    ~ButterLowPassFilt() = default;

    void apply3d(const float input[3], float output[3]);

private:
    float cutoff_freq_;
    float sample_freq_;
    float b0_, b1_, b2_;
    float a1_, a2_;

    // previous states
    float x_1[3] = {0.0f, 0.0f, 0.0f};
    float y_1[3] = {0.0f, 0.0f, 0.0f};
    float x_2[3] = {0.0f, 0.0f, 0.0f};
    float y_2[3] = {0.0f, 0.0f, 0.0f};

};


#endif // MATH_FILTER_BUTTER_LP_HPP