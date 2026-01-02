#include "gnc/state_estimation/ESKF.hpp"

namespace gnc {

    inline Eigen::Matrix3f getSkewSymmetric(const Eigen::Vector3f& vec) {
        Eigen::Matrix3f skew;
        skew <<     0.0f, -vec.z(),  vec.y(),
                 vec.z(),     0.0f, -vec.x(),
                -vec.y(),  vec.x(),     0.0f;
        return skew;
    }

    // print mag to serial
    inline void print3DUpdate(const Eigen::Vector3f& meas, const Eigen::Vector3f& pred, const Eigen::Vector3f& innov, const Eigen::Quaternionf& quat) {
        // clear screen on first update
        static bool first_print = true;
        if (first_print) {
            Serial.print("\e[2J"); // Clear screen
            first_print = false;
        }
        Serial.print("\e[H"); // Move cursor to top left
        Serial.println("EKF MONITOR");
        Serial.print("MEAS: ");
        Serial.print(meas.x(), 3); Serial.print(" ");
        Serial.print(meas.y(), 3); Serial.print(" ");
        Serial.println(meas.z(), 3);
        Serial.print("PRED: ");
        Serial.print(pred.x(), 3); Serial.print(" ");
        Serial.print(pred.y(), 3); Serial.print(" ");
        Serial.println(pred.z(), 3);
        Serial.print("INNO: ");
        Serial.print(innov.x(), 3); Serial.print(" ");
        Serial.print(innov.y(), 3); Serial.print(" ");
        Serial.println(innov.z(), 3);
        Serial.print("ATT: ");
        Serial.print(quat.x(), 4); Serial.print(" ");
        Serial.print(quat.y(), 4); Serial.print(" ");
        Serial.print(quat.z(), 4); Serial.print(" ");
        Serial.println(quat.w(), 4);
    }

    // print attitude error covariance to serial
    inline void printAttCovariance(const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& P) {
        static bool first_cov_print = true;
        if (first_cov_print) {
            Serial.print("\e[2J"); // Clear screen
            first_cov_print = false;
        }
        Serial.print("\e[H"); // Move cursor to top left
        Serial.println("EKF COVARIANCE MONITOR");

        // only print the dAtt rows and columns (3x3) at dTHETA_ID
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Serial.print(P(dTHETA_ID + i, dTHETA_ID + j), 6);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

}