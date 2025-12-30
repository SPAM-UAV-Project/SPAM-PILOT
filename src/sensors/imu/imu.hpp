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
}

#endif // IMU_HPP