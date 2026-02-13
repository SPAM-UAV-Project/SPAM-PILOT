/**
 * @file encoder.cpp
 * @brief Interfaces the AS5600 encoder to get angle measurements 
 */

#include "encoder.hpp"
#include "as5600.hpp"
#include <Arduino.h>
#include "pin_defs.hpp"
#include "msgs/EncoderMsg.hpp"
#include "timing/task_timing.hpp"
#include "filter/butter_lp.hpp"

namespace sensors::encoder
{
    AS5600 magEnc(I2C_ADDRESS_AS5600);
    static TwoWire magI2C = TwoWire(0);
    static TaskHandle_t encoderTaskHandle = NULL;
    static hw_timer_t* encoderTimer = NULL;

    // publisher
    Topic<EncoderMsg>::Publisher encoder_pub;

    void IRAM_ATTR onEncoderTimer() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // notify the encoder task to run
        vTaskNotifyGiveFromISR(encoderTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }

    void clearI2CBus(int sda, int scl) {
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, OUTPUT);

        // If SDA is stuck low, toggle SCL to force the slave to release it
        for (int i = 0; i < 16; i++) {
            digitalWrite(scl, LOW);
            delayMicroseconds(10);
            digitalWrite(scl, HIGH);
            delayMicroseconds(10);
            if (digitalRead(sda) == HIGH) break; // Bus is free
        }
    }

    void initEncoder()
    {
        // clearI2CBus(PIN_ENC_SDA, PIN_ENC_SCL);
        magI2C.begin(PIN_ENC_SDA, PIN_ENC_SCL);
        magI2C.setClock(400000);
        delay(100);
        
        // confirm I2C is working
        magI2C.beginTransmission(I2C_ADDRESS_AS5600);
        delay(10);
        uint8_t error = magI2C.endTransmission();
        if (error != 0) {
            Serial.println("[Encoder]: ERROR - Cannot communicate with AS5600!");
            return;
        }
        
        // start freertos tasks
        xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 4096, NULL, 6, &encoderTaskHandle, 1);
        delay(100);

        // create timer to trigger tasks
        Serial.println("[Encoder]: Setting up timer");
        encoderTimer = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(encoderTimer, &onEncoderTimer);
        timerAlarm(encoderTimer, 800, true, 0); //  alarm, auto-reload
        Serial.println("[Encoder]: Encoder initialized.");
    }

    void encoderTask(void *pvParameters)
    {
        // initialize AS5600 I2C comms
        magEnc.init(&magI2C);
        magI2C.setClock(1000000); // set to 1 MHz

        // configure the AS5600 for max speed
        AS5600Conf ASconf;
        ASconf.sf = 0b11; // slow filter
        ASconf.fth = 0b000; // fast threshold
        magEnc.setConf(ASconf); 
        magEnc.closeTransactions = false;

        // please Speed I need this my motor is kinda sensorless (ball knowledge reqd)
        float last_angle = 0.0;
        float last_time = micros();
        float current_time = 0.0;

        // test single read
        AS5600Conf regs = magEnc.readConf();
        Serial.println("[Encoder]: SF = " + String(regs.sf, BIN) + " FTH = " + String(regs.fth, BIN));

        // angle message
        EncoderMsg encoder_msg;

        // timing stuff
        // TaskTiming task_timer("Encoder", 800); // 800us budget for 1250Hz

        // filter 
        ButterLowPassFilt ang_vel_filt;
        ang_vel_filt.setup(15.0f, 1250.0f);

        while(1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // task_timer.startCycle();            
            // read angle
            float angle_rad = magEnc.readRawAngle() * AS5600_RAW_TO_RAD;
            
            // store in atomic for low jitter access
            atomic_enc_angle_rad.store(angle_rad, std::memory_order_relaxed);

            // compute angular velocity for dynamic notch filter
            current_time = micros();
            float delta_time_s = (current_time - last_time) * 1e-6f; // convert to seconds
            float delta_angle = angle_rad - last_angle;
            if (delta_angle > M_PI) {
                delta_angle -= 2.0f * M_PI;
            } else if (delta_angle < -M_PI) {
                delta_angle += 2.0f * M_PI;
            }
            last_angle = angle_rad; 
            last_time = current_time;

            // apply filter after finding out cutoffs
            ang_vel_filt.apply1d(delta_angle / delta_time_s, encoder_msg.angular_velocity_rad_s);
            
            // pack message
            encoder_msg.timestamp = current_time;
            // encoder_msg.angular_velocity_rad_s = delta_angle / delta_time_s;
            encoder_msg.angle_rad = angle_rad;
            encoder_pub.push(encoder_msg);

            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 1250 == 0) {
            //     task_timer.printStats();
            //     Serial.println("Angular Velocity (rad/s): " + String(encoder_msg.angular_velocity_rad_s));
            // }
        }
    }    
}