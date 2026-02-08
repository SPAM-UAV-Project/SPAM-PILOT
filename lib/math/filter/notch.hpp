#ifndef MATH_FILTER_NOTCH_HPP
#define MATH_FILTER_NOTCH_HPP

class NotchFilt
{
public:
    NotchFilt();
    ~NotchFilt() = default;

    void setup(double notch_freq, double bandwidth, double sample_freq);
    void update(float notch_freq, float bandwidth_half, float sample_freq_ang_inv);
    void apply3d(const float input[3], float output[3]);

private:
    float b0n_, b1n_, b2n_;
    float a1n_, a2n_;

    // previous states
    float x_1[3];
    float y_1[3];
    float x_2[3];
    float y_2[3];

};

#endif // MATH_FILTER_NOTCH_HPP