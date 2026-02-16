/***
 * Butterworth low pass filter implementation
 */

#ifndef MATH_FILTER_BUTTER_LP_HPP
#define MATH_FILTER_BUTTER_LP_HPP

class ButterLowPassFilt
{
public:
    ButterLowPassFilt();
    ~ButterLowPassFilt() = default;

    void setup(float cutoff_freq, float sample_freq);
    void apply3d(const float input[3], float output[3]);
    void apply1d(float input, float& output);
    void reset();

private:
    float b0_, b1_, b2_;
    float a1_, a2_;

    // previous states
    float x_1[3];
    float y_1[3];
    float x_2[3];
    float y_2[3];

};



#endif // MATH_FILTER_BUTTER_LP_HPP