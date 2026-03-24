#ifndef OPTICAL_FLOW_MSG_HPP
#define OPTICAL_FLOW_MSG_HPP

#include "msg_broker.hpp"

struct OpticalFlowMsg
{
    uint64_t timestamp = 0; // microseconds
    float distance = 0.0; // m
    uint8_t distance_precision = 0; // 0-255, lower better
    uint8_t distance_status = 0; // 0 = invalid, 1 = valid 
    float flow_vel_x = 0.0; // m/s
    float flow_vel_y = 0.0; // m/s
    uint8_t flow_quality = 0; // 0-255, higher better
    uint8_t flow_status = 0; // 0 = invalid, 1 = valid
};

#endif // OPTICAL_FLOW_MSG_HPP