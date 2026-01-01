#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"

#define MAG_LSB_2_uT 0.092f

namespace sensors::imu
{
    inline xTaskHandle imuTaskHandle = NULL;

    void imuISR();
    void initIMU();
    void imuTask(void *pvParameters);
    void magTask(void *pvParameters);

    // mag hard iron bias (uT)
    inline Eigen::Vector3f gyro_bias = {0.0f, 0.0f, 0.0f};
    inline Eigen::Vector3f accel_bias = {0.0f, 0.0f, 0.0f};
    inline Eigen::Vector3f mag_bias = {9.695457f, -9.284174f, 1.248736f};

    // gyro and accel transformation to FRD
    const Eigen::Matrix3f IMU_TO_FRD_ROT = (Eigen::Matrix3f() << 
        1.0f,  0.0f,  0.0f,
        0.0f,  -1.0f,  0.0f,
        0.0f,  0.0f, -1.0f).finished();

    // mag transformation to FRD (mag sensor is +X -> West, +Y -> South, +Z -> down)
    const Eigen::Matrix3f MAG_TO_FRD_ROT = (Eigen::Matrix3f() << 
        0.0f,  -1.0f,  0.0f,
        -1.0f,  0.0f,  0.0f,
        0.0f,  0.0f, 1.0f).finished();

    // imu orientation w.r.t body
    const Eigen::Matrix3f IMU_TO_BODY_ROT = (Eigen::Matrix3f() << 
        1.0f,  0.0f,  0.0f,
        0.0f,  1.0f,  0.0f,
        0.0f,  0.0f, 1.0f).finished();

    // mag orientation w.r.t body
    const Eigen::Matrix3f MAG_TO_BODY_ROT = (Eigen::Matrix3f() << 
        1.0f,  0.0f,  0.0f,
        0.0f,  1.0f,  0.0f,
        0.0f,  0.0f, 1.0f).finished();
}

#endif // IMU_HPP