#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>

namespace sensors::imu
{
    inline xTaskHandle imuTaskHandle = NULL;

    void IRAM_ATTR imuISR();
    void initIMU();
    void imuTask(void *pvParameters);
}

#endif // IMU_HPP