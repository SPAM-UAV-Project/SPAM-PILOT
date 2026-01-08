/**
 * @file transport_base.hpp
 * @brief Abstract transport interface for MAVLink communication
 */

#ifndef TRANSPORT_BASE_HPP
#define TRANSPORT_BASE_HPP

#include <Arduino.h>

namespace cdh::mavlink {

/**
 * @brief Abstract base class for MAVLink transport implementations
 */
class TransportBase {
public:
    virtual ~TransportBase() = default;
    virtual bool begin() = 0;
    virtual void end() = 0;
    virtual bool isConnected() = 0;
    virtual size_t available() = 0;
    virtual size_t receive(uint8_t* buffer, size_t max_len) = 0;
    virtual size_t send(const uint8_t* data, size_t len) = 0;
};

} // namespace cdh::mavlink

#endif // TRANSPORT_BASE_HPP
