#ifndef MODEL_SECOND_ORDER_ACTUATOR_HPP
#define MODEL_SECOND_ORDER_ACTUATOR_HPP

#include "ArduinoEigen/Eigen/Dense"


class SecondOrderActuator {
public:
    struct Coeffs { float b0, b1, b2, a1, a2; };

    // Function to calculate discrete coefficients from continuous s-domain
    // H(s) = (B1*s + B2) / (s^2 + A1*s + A2)  <-- Matching your image data
    static Coeffs discretize(float B1, float B2, float A1, float A2, float dt) {
        double T = (double)dt;
        // Tustin substitution constants
        double alpha = 2.0 / T;
        double alpha2 = alpha * alpha;
        
        double den = alpha2 + A1 * alpha + A2;
        
        Coeffs c;
        c.b0 = (float)((B1 * alpha + B2) / den);
        c.b1 = (float)((2.0 * B2) / den);
        c.b2 = (float)((B2 - B1 * alpha) / den);
        c.a1 = (float)((2.0 * A2 - 2.0 * alpha2) / den);
        c.a2 = (float)((alpha2 - A1 * alpha + A2) / den);
        return c;
    }

    void setup(Coeffs x_coeffs, Coeffs y_coeffs, Coeffs z_coeffs) {
        c_[0] = x_coeffs; c_[1] = y_coeffs; c_[2] = z_coeffs;
    }

    Eigen::Vector3f update(const Eigen::Vector3f& input) {
        Eigen::Vector3f output;
        for (int i = 0; i < 3; ++i) {
            output[i] = c_[i].b0 * input[i] + c_[i].b1 * x1_[i] + c_[i].b2 * x2_[i] 
                        - c_[i].a1 * y1_[i] - c_[i].a2 * y2_[i];
            // State shift
            x2_[i] = x1_[i]; x1_[i] = input[i];
            y2_[i] = y1_[i]; y1_[i] = output[i];
        }
        return output;
    }

    void reset() {
        x1_.setZero();
        x2_.setZero();
        y1_.setZero();
        y2_.setZero();
    }

private:
    Coeffs c_[3];
    Eigen::Vector3f x1_{0,0,0}, x2_{0,0,0}, y1_{0,0,0}, y2_{0,0,0};
};

#endif // MODEL_SECOND_ORDER_ACTUATOR_HPP