#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"

namespace sensors::imu
{
    inline xTaskHandle imuTaskHandle = NULL;

    inline Eigen::Vector3f accel_data;
    inline Eigen::Vector3f gyro_data;
    inline Eigen::Vector3f mag_data; // placeholder for future magnetometer

    void imuISR();
    void initIMU();
    void imuTask(void *pvParameters);
}

#endif // IMU_HPP