#ifndef ENCODER_MSG_HPP
#define ENCODER_MSG_HPP

#include "msg_broker.hpp"

/**
 * @param timestamp Timestamp in microseconds
 * @param angle_rad Angle in radians
 * @param angular_velocity_rad_s Angular velocity in radians per second 
 */
struct EncoderMsg
{
    uint64_t timestamp = 0;
    float angle_rad = 0.0f;
    float angular_velocity_rad_s = 0.0f;
};

#endif // ENCODER_MSG_HPP


