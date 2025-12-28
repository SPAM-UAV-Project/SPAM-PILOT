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

    void initEncoder()
    {
        magI2C.begin(PIN_ENC_SDA, PIN_ENC_SCL);
        magI2C.setClock(400000);
        
        // confirm I2C is working
        magI2C.beginTransmission(I2C_ADDRESS_AS5600);
        delay(100);
        uint8_t error = magI2C.endTransmission();
        if (error != 0) {
            Serial.println("[Encoder]: ERROR - Cannot communicate with AS5600!");
            return;
        }
        
        // start freertos tasks
        xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 3, &encoderTaskHandle);

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

        // test single read
        AS5600Conf regs = magEnc.readConf();
        Serial.println("[Encoder]: SF = " + String(regs.sf, BIN) + " FTH = " + String(regs.fth, BIN));

        // angle message
        EncoderMsg encoder_msg;

        while(1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            float angle_rad = magEnc.readRawAngle() * AS5600_RAW_TO_RAD;
            encoder_msg.timestamp = micros();
            encoder_msg.angle_rad = angle_rad;
            encoder_pub.push(encoder_msg);
        }
    }    
}