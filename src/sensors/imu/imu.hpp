#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"

namespace sensors::imu
{
    inline xTaskHandle imuTaskHandle = NULL;

    void imuISR();
    void initIMU();
    void imuTask(void *pvParameters);
}

#endif // IMU_HPP