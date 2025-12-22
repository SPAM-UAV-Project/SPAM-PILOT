#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <atomic>

#define PIN_ENC_SDA 6
#define PIN_ENC_SCL 7 // haha funny number
#define PIN_ENC_DIR 4 // direction on pin 4
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