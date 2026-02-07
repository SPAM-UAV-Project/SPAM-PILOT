#include "cdh/radio_controller/rc.hpp"
// #include "timing/task_timing.hpp"

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
        const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz polling rate
        
        // NOTE: Don't call crsfSerial_.begin() here - CRSFforArduino handles it internally
        crsf_.setRcChannelsCallback(onChannelsReceivedStatic);
        
        if (!crsf_.begin(crsf_baud_)) 
        {
            crsf_.end();
            Serial.println("CRSF for Arduino initialisation failed!");
        }
        else
        {
            Serial.println("[RC] CRSF initialized successfully");
        }

        // TaskTiming task_timer("RC Poll", 20000); // 20000us budget for 50Hz

        while (true) {
            // task_timer.startCycle();

            // Update attitude telemetry at 10Hz (must be set BEFORE update() for transmission)
            uint32_t now = millis();
            if (now - last_telemetry_time_ >= TELEMETRY_PERIOD_MS) {
                last_telemetry_time_ = now;

                if (ekf_sub_.pull_if_new(ekf_msg_)) {
                    // Convert quaternion to euler angles (FRD frame)
                    // EKF uses FRD: X=Forward, Y=Right, Z=Down
                    const auto& q = ekf_msg_.attitude;
                    
                    // Roll (rotation around X/forward axis)
                    float sinr_cosp = 2.0f * (q.w() * q.x() + q.y() * q.z());
                    float cosr_cosp = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
                    float roll_rad = atan2f(sinr_cosp, cosr_cosp);
                    
                    // Pitch (rotation around Y/right axis)
                    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
                    float pitch_rad = asinf(fminf(fmaxf(sinp, -1.0f), 1.0f));
                    
                    // Yaw (rotation around Z/down axis)
                    float siny_cosp = 2.0f * (q.w() * q.z() + q.x() * q.y());
                    float cosy_cosp = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
                    float yaw_rad = atan2f(siny_cosp, cosy_cosp);
                    
                    // Convert to decidegrees for CRSF (x10)
                    // Negate pitch so positive = nose up (tilt back)
                    int16_t roll_ddeg = static_cast<int16_t>(roll_rad * 572.9578f);
                    int16_t pitch_ddeg = static_cast<int16_t>(-pitch_rad * 572.9578f);
                    int16_t yaw_ddeg = static_cast<int16_t>(yaw_rad * 572.9578f);
                    if (yaw_ddeg < 0) yaw_ddeg += 3600;  // 0-360 range
                    
                    crsf_.telemetryWriteAttitude(roll_ddeg, pitch_ddeg, yaw_ddeg);
                }
            }

            // Process incoming CRSF frames and send telemetry responses
            crsf_.update();

            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 50 == 0) {
            //     task_timer.printStats();
            // }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

}