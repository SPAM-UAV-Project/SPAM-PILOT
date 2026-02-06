#include "cdh/radio_controller/rc.hpp"

namespace cdh {
    
    RadioController* RadioController::instance_ = nullptr;

    RadioController::RadioController() 
    {
        instance_ = this;
    };

    void RadioController::init() 
    {
        xTaskCreatePinnedToCore(
            rcPollEntryTask,
            "RC Poll Task",
            4096,
            this,
            3,
            nullptr,
            0
        );
    }

    void RadioController::onChannelsReceived(serialReceiverLayer::rcChannels_t *rcChannels) 
    {
        // handle if controller disconnects
        if (rcChannels->failsafe == true) {
            rcCommandPub_.push(failsafeCommandMsg_);
        } else {
            // map RC channels and publish
            // if controller is not changed, we have the following mapping:
            // 0 - Roll, 1 - Pitch, 2 - Throttle, 3 - Yaw

            roll_ = (static_cast<float>(crsf_.rcToUs(crsf_.getChannel(1)) - 1500.0f) / 500.0f);    // -1 to 1
            pitch_ = (static_cast<float>(crsf_.rcToUs(crsf_.getChannel(2)) - 1500.0f) / 500.0f);   // -1 to 1
            throttle_ = (static_cast<float>(crsf_.rcToUs(crsf_.getChannel(3)) - 1000.0f) / 1000.0f); // 0 to 1
            yaw_ = (static_cast<float>(crsf_.rcToUs(crsf_.getChannel(4)) - 1500.0f) / 500.0f);    // -1 to 1
            // clamp values
            roll_ = fmaxf(fminf(roll_, 1.0f), -1.0f);
            pitch_ = fmaxf(fminf(pitch_, 1.0f), -1.0f);
            yaw_ = fmaxf(fminf(yaw_, 1.0f), -1.0f);
            throttle_ = fmaxf(fminf(throttle_, 1.0f), 0.0f);

            rcCommandMsg_.timestamp = micros();
            rcCommandMsg_.roll = roll_;
            rcCommandMsg_.pitch = -pitch_; // invert pitch axis
            rcCommandMsg_.yaw = yaw_;
            rcCommandMsg_.throttle = throttle_;
            rcCommandMsg_.arm_switch = (rcChannels->value[4] > 1500);
            rcCommandMsg_.emergency_stop = (rcChannels->value[5] > 1500);
            rcCommandPub_.push(rcCommandMsg_);
        }
    }

    void RadioController::pollRCTask(void *pvParameters) 
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz

        crsfSerial_.begin(crsf_baud_, SERIAL_8N1, crsf_rxPin, crsf_txPin, crsf_invert);
        crsf_.setRcChannelsCallback(onChannelsReceivedStatic);
        
        if (!crsf_.begin(crsf_baud_)) 
        {
            crsf_.end();
            Serial.println("CRSF for Arduino initialisation failed!");
        }

        while (true) {
            crsf_.update();

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

}