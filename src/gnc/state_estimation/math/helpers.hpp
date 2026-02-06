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
        //clear screen on first update
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

        // convert quaternion to euler for printing
        Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        Serial.print("EULER (deg): ");
        Serial.print(euler.x() * RAD_TO_DEG, 2); Serial.print(" ");
        Serial.print(euler.y() * RAD_TO_DEG, 2); Serial.print(" ");
        Serial.println(euler.z() * RAD_TO_DEG, 2);
    }

    // print attitude error covariance to serial
    inline void printAttCovariance(const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& P) {
        static bool first_cov_print = true;
        if (first_cov_print) {
            Serial.print("\e[2J"); // Clear screen
            first_cov_print = false;
        }
        Serial.print("\e[H"); // Move cursor to top left
        Serial.println("ATTITUDE COVARIANCE");
        Serial.print("P_THETA_THETA:\n");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Serial.print(P(i + dTHETA_ID, j + dTHETA_ID), 6);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    inline void printInnovCovariance(const Eigen::Matrix<float, 3, 3>& mat) {
        Serial.println("3x3 Matrix:");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Serial.print(mat(i, j), 6);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

}