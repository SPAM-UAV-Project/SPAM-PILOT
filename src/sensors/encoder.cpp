#include "encoder.hpp"
#include "as5600.hpp"
#include <Arduino.h>

namespace sensors::encoder
{
    AS5600 magEnc(I2C_ADDRESS_AS5600);
    static TwoWire magI2C = TwoWire(0);
    static TaskHandle_t encoderTaskHandle = NULL;

    static hw_timer_t* encoderTimer = NULL;

#ifdef LOG_ENCODER
    // circular logging buffer
    #define LOG_SIZE 1000
    static float log_buffer[LOG_SIZE];
    static uint32_t log_timestamps[LOG_SIZE];
    static volatile uint16_t log_index = 0;
    static volatile bool log_full = false;

    static TaskHandle_t logTaskHandle = NULL;
#endif

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
#ifdef LOG_ENCODER
        xTaskCreate(encoderLoggerTask, "LogTask", 4096, NULL, 1, &logTaskHandle);
#endif


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
        ASconf.sf = 0b11;
        ASconf.fth = 0b000;
        magEnc.setConf(ASconf); 
        magEnc.closeTransactions = false;

        // test single read
        AS5600Conf regs = magEnc.readConf();
        Serial.println("[Encoder]: SF = " + String(regs.sf, BIN) + " FTH = " + String(regs.fth, BIN));

        while(1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            float angle_rad = magEnc.readRawAngle() * AS5600_RAW_TO_RAD;
            enc_angle_rad.store(angle_rad, std::memory_order_relaxed);

#ifdef LOG_ENCODER
            log_buffer[log_index] = angle_rad;
            log_timestamps[log_index] = millis();
            log_index++;
            
            // notify log task when buffer is full
            if (log_index >= LOG_SIZE) {
                log_index = 0;
                xTaskNotifyGive(logTaskHandle);
            }
#endif
            
        }
    }

#ifdef LOG_ENCODER
    void encoderLoggerTask(void *pvParameters)
    {
        static float local_buffer[LOG_SIZE];
        static uint32_t local_timestamps[LOG_SIZE];
        Serial.println("Timestamp(ms),Angle(rad)");

        while(1) {
            // Block until notified by encoder task
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // copy log locally (note this may contain partially corrupted data due to task preemption but ok for now)
            memcpy(local_buffer, (void*)log_buffer, sizeof(log_buffer));
            memcpy(local_timestamps, (void*)log_timestamps, sizeof(log_timestamps));
            // Print all data
            for (uint16_t i = 0; i < LOG_SIZE; i++) {
                Serial.printf("%lu,%.4f\n", 
                             local_timestamps[i], 
                             local_buffer[i]);
            }
        }
    }
#endif
    
}