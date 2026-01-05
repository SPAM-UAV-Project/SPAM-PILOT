/**
 * @file wifi_transport.hpp
 * @brief WiFi Station + TCP client transport for MAVLink communication
 */

#ifndef WIFI_TRANSPORT_HPP
#define WIFI_TRANSPORT_HPP

#include "transport_base.hpp"
#include <WiFi.h>

namespace cdh::mavlink {

class WifiTransport : public TransportBase {
public:
    WifiTransport(const char* ssid = "tommy", 
                  const char* password = "hilfiger",
                  const char* server_ip = "192.168.1.100",  // GCS IP (adjust as needed)
                  uint16_t port = 14550)
        : ssid_(ssid), password_(password), server_ip_(server_ip), port_(port) {}

    ~WifiTransport() { end(); }

    bool begin() override {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid_, password_);
        
        Serial.printf("[WifiTransport] Connecting to %s...\n", ssid_);
        
        // Wait for connection (with timeout)
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[WifiTransport] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            initialized_ = true;
            return true;
        } else {
            Serial.println("\n[WifiTransport] Connection failed");
            return false;
        }
    }

    void end() override {
        if (client_) client_.stop();
        WiFi.disconnect(true);
        initialized_ = false;
    }

    bool isConnected() override { 
        return initialized_ && client_ && client_.connected(); 
    }

    size_t available() override {
        tryConnect();
        return (client_ && client_.connected()) ? client_.available() : 0;
    }

    size_t receive(uint8_t* buffer, size_t max_len) override {
        if (!client_ || !client_.connected()) return 0;
        size_t count = 0;
        while (count < max_len && client_.available() > 0) {
            buffer[count++] = client_.read();
        }
        return count;
    }

    size_t send(const uint8_t* data, size_t len) override {
        if (!client_ || !client_.connected()) return 0;
        return client_.write(data, len);
    }

private:
    void tryConnect() {
        if (!initialized_ || WiFi.status() != WL_CONNECTED) return;
        
        // Try to connect to GCS if not connected
        if (!client_ || !client_.connected()) {
            if (client_.connect(server_ip_, port_)) {
                Serial.printf("[WifiTransport] Connected to GCS %s:%d\n", server_ip_, port_);
            }
        }
    }

    const char* ssid_;
    const char* password_;
    const char* server_ip_;
    uint16_t port_;
    WiFiClient client_;
    bool initialized_ = false;
};

} // namespace cdh::mavlink

#endif // WIFI_TRANSPORT_HPP
