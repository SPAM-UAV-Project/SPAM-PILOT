/**
 * @file sym_minimal.h
 * @brief Minimal standalone implementation of sym::Rot3 for embedded use
 * This replaces the full symforce library with just what's needed for predict_states()
 */

#pragma once

#include <ArduinoEigen/Eigen/Core>
#include <ArduinoEigen/Eigen/Geometry>

namespace sym {

/**
 * Minimal Rot3 class compatible with generated predict_states function.
 * Only implements what's needed - no storage ops, type enums, etc.
 */
template <typename Scalar>
class Rot3 {
public:
    using DataVec = Eigen::Matrix<Scalar, 4, 1>;

    // Default constructor (identity)
    Rot3() : data_(Scalar(0), Scalar(0), Scalar(0), Scalar(1)) {}

    // Construct from quaternion coefficients [x,y,z,w]
    explicit Rot3(const DataVec& data, bool normalize = true)
        : data_(normalize ? data.normalized() : data) {}

    // Construct from Eigen::Quaternion
    explicit Rot3(const Eigen::Quaternion<Scalar>& q)
        : data_(q.coeffs()) {}

    // Static factory from quaternion
    static Rot3 FromQuaternion(const Eigen::Quaternion<Scalar>& q) {
        return Rot3(q);
    }

    // Access underlying data (quaternion coefficients [x,y,z,w])
    const DataVec& Data() const { return data_; }

    // Convert to Eigen::Quaternion
    Eigen::Quaternion<Scalar> Quaternion() const {
        return Eigen::Quaternion<Scalar>(data_);
    }

    // Multiply two rotations (quaternion multiplication)
    Rot3 operator*(const Rot3& other) const {
        return Rot3(Quaternion() * other.Quaternion());
    }

    // Rotate a vector (apply rotation to vector)
    Eigen::Matrix<Scalar, 3, 1> operator*(const Eigen::Matrix<Scalar, 3, 1>& vec) const {
        return Quaternion() * vec;
    }

    // Create rotation from tangent vector (axis-angle representation)
    static Rot3 FromTangent(const Eigen::Matrix<Scalar, 3, 1>& tangent, Scalar epsilon = Scalar(1e-8)) {
        const Scalar angle = tangent.norm();
        if (angle < epsilon) {
            return Rot3();  // Identity for small angles
        }
        const Eigen::Matrix<Scalar, 3, 1> axis = tangent / angle;
        return Rot3(Eigen::Quaternion<Scalar>(Eigen::AngleAxis<Scalar>(angle, axis)));
    }

    // Convert to tangent vector (axis-angle representation)
    Eigen::Matrix<Scalar, 3, 1> ToTangent(Scalar epsilon = Scalar(1e-8)) const {
        const Eigen::AngleAxis<Scalar> aa(Quaternion());
        const Scalar angle = aa.angle();
        if (angle < epsilon) {
            return Eigen::Matrix<Scalar, 3, 1>::Zero();
        }
        return angle * aa.axis();
    }

    // Get rotation matrix
    Eigen::Matrix<Scalar, 3, 3> ToRotationMatrix() const {
        return Quaternion().toRotationMatrix();
    }

private:
    DataVec data_;  // Quaternion coefficients [x,y,z,w]
};

} // namespace sym
