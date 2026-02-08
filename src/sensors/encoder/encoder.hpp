/**
 * @file encoder.hpp
 * @brief Header file for AS5600 encoder interface 
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <atomic>
#define I2C_ADDRESS_AS5600 0x36

namespace sensors::encoder
{
    constexpr float AS5600_RAW_TO_RAD  = 0.001533980788f; //(2.0f * M_PI / 4096.0f);

    // Atomic encoder angle for direct access from rotor control (no pub/sub jitter)
    inline std::atomic<float> atomic_enc_angle_rad;
    
    void initEncoder();
    void encoderTask(void *pvParameters);
    void encoderLoggerTask(void *pvParameters);
}

#endif // ENCODER_HPP