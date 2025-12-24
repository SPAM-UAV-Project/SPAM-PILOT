#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <atomic>

#define I2C_ADDRESS_AS5600 0x36

#define AS5600_RAW_TO_RAD (2.0f * M_PI / 4096.0f)

//#define LOG_ENCODER // enable encoder logging task

namespace sensors::encoder
{
    inline std::atomic<float> enc_angle_rad;

    void initEncoder();
    void encoderTask(void *pvParameters);
    void encoderLoggerTask(void *pvParameters);
}

#endif // ENCODER_HPP