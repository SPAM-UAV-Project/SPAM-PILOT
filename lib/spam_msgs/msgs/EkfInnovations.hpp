#ifndef EKF_INNOVATIONS_MSG_HPP
#define EKF_INNOVATIONS_MSG_HPP

#include "msg_broker.hpp"
#include "ArduinoEigen/Eigen/Dense"

/**
 * @param timestamp Timestamp in microseconds
 * @param mag_innov Magnetometer innovation
 * @param mag_innov_cov Magnetometer innovation covariance
 * @param gravity_innov Gravity innovation
 * @param gravity_innov_cov Gravity innovation covariance
 */
struct EkfInnovationsMsg
{
    uint64_t timestamp = 0;
    Eigen::Vector3f mag_innov = Eigen::Vector3f::Zero();
    Eigen::Matrix3f mag_innov_cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f gravity_innov = Eigen::Vector3f::Zero();
    Eigen::Matrix3f gravity_innov_cov = Eigen::Matrix3f::Zero();
};

#endif // EKF_INNOVATIONS_MSG_HPP


