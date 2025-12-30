#include <ArduinoEigen/Eigen/Dense>

namespace gnc {

    Eigen::Matrix3f getSkewSymmetric(const Eigen::Vector3f& vec) {
        Eigen::Matrix3f skew;
        skew <<     0.0f, -vec.z(),  vec.y(),
                 vec.z(),     0.0f, -vec.x(),
                -vec.y(),  vec.x(),     0.0f;
        return skew;
    }

}