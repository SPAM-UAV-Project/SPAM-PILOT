/**
 * @file usb_transport.hpp
 * @brief USB Serial transport for MAVLink communication
 */

#ifndef USB_TRANSPORT_HPP
#define USB_TRANSPORT_HPP

#include "transport_base.hpp"

namespace cdh::mavlink {

class UsbTransport : public TransportBase {
public:
    explicit UsbTransport(HardwareSerial& serial = Serial, uint32_t baud = 921600)
        : serial_(serial), baud_rate_(baud) {}

    bool begin() override {
        serial_.begin(baud_rate_);
        initialized_ = true;
        return true;
    }

    void end() override {
        serial_.end();
        initialized_ = false;
    }

    bool isConnected() override { return initialized_ && serial_; }
    size_t available() override { return serial_.available(); }

    size_t receive(uint8_t* buffer, size_t max_len) override {
        size_t count = 0;
        while (count < max_len && serial_.available() > 0) {
            buffer[count++] = serial_.read();
        }
        return count;
    }

    size_t send(const uint8_t* data, size_t len) override {
        return serial_.write(data, len);
    }

private:
    HardwareSerial& serial_;
    uint32_t baud_rate_;
    bool initialized_ = false;
};

} // namespace cdh::mavlink

#endif // USB_TRANSPORT_HPP
