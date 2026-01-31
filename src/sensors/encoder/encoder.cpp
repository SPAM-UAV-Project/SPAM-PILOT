/**
 * @file encoder.cpp
 * @brief Interfaces the AS5600 encoder to get angle measurements 
 */

#include "encoder.hpp"
#include "as5600.hpp"
#include <Arduino.h>
#include "pin_defs.hpp"
#include "msgs/EncoderMsg.hpp"

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
        xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 4096, NULL, 3, &encoderTaskHandle, 0);
        delay(100);

        // create timer to trigger tasks
        Serial.println("[Encoder]: Setting up timer");
        encoderTimer = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(encoderTimer, &onEncoderTimer);
        timerAlarm(encoderTimer, 1000, true, 0); // 1000 Hz alarm, auto-reload
        Serial.println("[Encoder]: Encoder initialized.");
    }

    void encoderTask(void *pvParameters)
    {
        // initialize AS5600 I2C comms
        magEnc.init(&magI2C);

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

        while(1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // read angle
            float angle_rad = magEnc.readRawAngle() * AS5600_RAW_TO_RAD;
            
            // store in atomic for low jitter access
            atomic_enc_angle_rad.store(angle_rad, std::memory_order_relaxed);

            // compute angular velocity (don't really need at this stage)
            // current_time = micros();
            // float delta_time_s = (current_time - last_time) / 1000000.0f;
            // float delta_angle = angle_rad - last_angle;
            // if (delta_angle > M_PI) {
            //     delta_angle -= 2.0f * M_PI;
            // } else if (delta_angle < -M_PI) {
            //     delta_angle += 2.0f * M_PI;
            // }
            // encoder_msg.angular_velocity_rad_s = delta_angle / delta_time_s;
            // last_angle = angle_rad; 
            // last_time = current_time;

            encoder_msg.angle_rad = angle_rad;
            encoder_pub.push(encoder_msg);
        }
    }    
}