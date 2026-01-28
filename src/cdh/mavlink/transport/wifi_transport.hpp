/**
 * @file wifi_transport.hpp
 * @brief WiFi Station + UDP broadcast transport for MAVLink communication
 */

#ifndef WIFI_TRANSPORT_HPP
#define WIFI_TRANSPORT_HPP

#ifdef WIFI_TRANSPORT
#include "transport_base.hpp"
#include <WiFi.h>
#include <WiFiUdp.h>

namespace cdh::mavlink {

class WifiTransport : public TransportBase {
public:
    WifiTransport(const char* ssid = "tommy", 
                  const char* password = "hilfiger",
                  uint16_t port = 14550)
        : ssid_(ssid), password_(password), port_(port) {}

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
            
            // Start UDP
            if (udp_.begin(port_)) {
                Serial.printf("[WifiTransport] UDP started on port %d (broadcasting)\n", port_);
                initialized_ = true;
                
                // Calculate broadcast address
                IPAddress ip = WiFi.localIP();
                IPAddress subnet = WiFi.subnetMask();
                broadcast_ip_ = IPAddress(
                    ip[0] | (~subnet[0]),
                    ip[1] | (~subnet[1]),
                    ip[2] | (~subnet[2]),
                    ip[3] | (~subnet[3])
                );
                Serial.printf("[WifiTransport] Broadcasting to %s:%d\n", 
                    broadcast_ip_.toString().c_str(), port_);
                
                return true;
            } else {
                Serial.println("[WifiTransport] UDP begin failed");
                return false;
            }
        } else {
            Serial.println("\n[WifiTransport] Connection failed");
            return false;
        }
    }

    void end() override {
        udp_.stop();
        WiFi.disconnect(true);
        initialized_ = false;
    }

    bool isConnected() override { 
        return initialized_ && WiFi.status() == WL_CONNECTED; 
    }

    size_t available() override {
        if (!initialized_) return 0;
        
        // Cache packet size to avoid redundant parsePacket() calls
        if (cached_packet_size_ == 0) {
            cached_packet_size_ = udp_.parsePacket();
        }
        return cached_packet_size_;
    }

    size_t receive(uint8_t* buffer, size_t max_len) override {
        if (!initialized_) return 0;
        
        // Use cached packet size if available
        if (cached_packet_size_ == 0) {
            cached_packet_size_ = udp_.parsePacket();
        }
        
        if (cached_packet_size_ == 0) return 0;
        
        size_t to_read = (cached_packet_size_ < max_len) ? cached_packet_size_ : max_len;
        size_t bytes_read = udp_.read(buffer, to_read);
        
        // Clear cache after reading
        cached_packet_size_ = 0;
        
        return bytes_read;
    }

    size_t send(const uint8_t* data, size_t len) override {
        if (!initialized_) return 0;
        
        // Broadcast to subnet - single call minimizes overhead
        udp_.beginPacket(broadcast_ip_, port_);
        size_t written = udp_.write(data, len);
        udp_.endPacket();
        
        return written;
    }

private:
    const char* ssid_;
    const char* password_;
    uint16_t port_;
    WiFiUDP udp_;
    IPAddress broadcast_ip_;
    bool initialized_ = false;
    int cached_packet_size_ = 0;  // Cache to avoid redundant parsePacket() calls
};

} // namespace cdh::mavlink
#endif // WIFI_TRANSPORT

#endif // WIFI_TRANSPORT_HPP
