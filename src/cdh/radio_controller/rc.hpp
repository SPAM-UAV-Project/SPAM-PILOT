#ifndef RC_HPP
#define RC_HPP

#include <Arduino.h>
#include "CRSFforArduino.hpp"
#include "msgs/RcCommandMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"

namespace cdh {

    class RadioController
    {
    public:
        RadioController();
        ~RadioController() = default;

        void init();

        int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
        const char *rcChannelNames[16] = 
        {
            "R",
            "P",
            "T",
            "Y",
            "Aux1",
            "Aux2",
            "Aux3",
            "Aux4",
            "Aux5",
            "Aux6",
            "Aux7",
            "Aux8",
            "Aux9",
            "Aux10",
            "Aux11",
            "Aux12"
        };

    private:

        void onChannelsReceived(serialReceiverLayer::rcChannels_t *rcChannels);
        static void onChannelsReceivedStatic(serialReceiverLayer::rcChannels_t *rcChannels) {
            if (instance_ != nullptr) {
                instance_->onChannelsReceived(rcChannels);
            }
        }

        void pollRCTask(void *pvParameters);
        static void rcPollEntryTask(void* instance) {
            static_cast<RadioController*>(instance)->pollRCTask(instance);
        }

        static RadioController* instance_;
        HardwareSerial crsfSerial_ = HardwareSerial(1);
        static constexpr int crsf_rxPin = 14;
        static constexpr int crsf_txPin = 13;
        CRSFforArduino crsf_{&crsfSerial_, crsf_rxPin, crsf_txPin};
        int crsf_baud_ = 420000;
        bool crsf_invert = false;

        // publisher
        float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f, throttle_ = 0.0f;
        Topic<RcCommandMsg>::Publisher rcCommandPub_;
        RcCommandMsg rcCommandMsg_;
        RcCommandMsg failsafeCommandMsg_; // initialized to defaults

        // Attitude telemetry
        Topic<EkfStatesMsg>::Subscriber ekf_sub_;
        EkfStatesMsg ekf_msg_;
        uint32_t last_telemetry_time_ = 0;
        static constexpr uint32_t TELEMETRY_PERIOD_MS = 100; // 10Hz
    };

}

#endif // RC_HPP