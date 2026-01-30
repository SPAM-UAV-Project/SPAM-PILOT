/**
 * @file imu.cpp
 * @brief implementation for IMU interface and tasks
 */

#include "imu.hpp"
#include "hmc5883l.hpp"
#include "MPU6050.h"
#include "pin_defs.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ImuMagMsg.hpp"

namespace sensors::imu
{
    TwoWire imuI2C = TwoWire(1);  // use I2C bus 1 for IMU

    MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &imuI2C);
    HMC5883L mag;
    Topic<ImuHighRateMsg>::Publisher imu_pub;
    Topic<ImuMagMsg>::Publisher mag_pub;

    // mutex for i2c reads
    SemaphoreHandle_t i2c_mutex;

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

    void initIMU()
    {
        i2c_mutex = xSemaphoreCreateMutex();

        clearI2CBus(PIN_IMU_SDA, PIN_IMU_SCL);
        imuI2C.begin(PIN_IMU_SDA, PIN_IMU_SCL);
        imuI2C.setClock(400000); // 400kHz 

        // setup MPU6050
        mpu.initialize();
        delay(100);

        if (!mpu.testConnection()) {
            Serial.println("[IMU]: ERROR - Cannot communicate with MPU6050!");
            return;
        }

        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
        mpu.setDLPFMode(0);
        mpu.setRate(7); // 1000 Hz from 8000 Hz / (1 + 7)

        mpu.setIntDataReadyEnabled(true);
        mpu.setInterruptMode(false);
        mpu.setInterruptDrive(false);
        mpu.setInterruptLatch(false);
        mpu.setInterruptLatchClear(true);
        mpu.setI2CBypassEnabled(true); // enable direct access to mag via i2c
        mpu.setI2CMasterModeEnabled(false);
        mpu.setSleepEnabled(false);
        xTaskCreate(imuTask, "IMU Task", 8192, NULL, 3, &imuTaskHandle);

        delay(100);

        // setup HMC5883L - pass the imuI2C instance
        if (!mag.begin(&imuI2C)) {
            Serial.println("[IMU]: ERROR - Cannot communicate with HMC5883L!");
            return;
        }
        mag.setRange(HMC5883L_RANGE_1_3GA);          // 1.3 Gauss
        mag.setMeasurementMode(HMC5883L_CONTINOUS); 
        mag.setDataRate(HMC5883L_DATARATE_30HZ);
        mag.setSamples(HMC5883L_SAMPLES_8); // oversampling
        xTaskCreate(magTask, "Mag Task", 8192, NULL, 3, NULL);

        // start interrupts
        pinMode(PIN_IMU_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuISR, RISING);

    }

    void IRAM_ATTR imuISR()
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void imuTask(void *pvParameters)
    {
        int16_t imu_raw[6] = {0};
        ImuHighRateMsg imu_msg;

        // low pass filter
        Eigen::Vector3f filtered_gyro = Eigen::Vector3f::Zero();
        float f_c = 25.0f; // cutoff freq
        float f_s = 1000.0f; // sample freq
        Eigen::Vector3f alpha_lp = (1- expf(-2*M_PI*f_c / f_s)) * Eigen::Vector3f::Ones();

        while (1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // read imu data
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                mpu.getMotion6(&imu_raw[0], &imu_raw[1], &imu_raw[2], &imu_raw[3], &imu_raw[4], &imu_raw[5]);
                xSemaphoreGive(i2c_mutex);
            }

            imu_msg.timestamp = micros();
            imu_msg.accel << imu_raw[0] / 8192.0f * 9.81f,
                             imu_raw[1] / 8192.0f * 9.81f,
                             imu_raw[2] / 8192.0f * 9.81f;
            imu_msg.gyro << (imu_raw[3] / 65.5f) * DEG_TO_RAD,
                            (imu_raw[4] / 65.5f) * DEG_TO_RAD,
                            (imu_raw[5] / 65.5f) * DEG_TO_RAD;
            // rotate into FRD
            imu_msg.gyro = IMU_TO_BODY_ROT * IMU_TO_FRD_ROT * imu_msg.gyro;
            imu_msg.accel = IMU_TO_BODY_ROT * IMU_TO_FRD_ROT * imu_msg.accel;

            filtered_gyro = alpha_lp.asDiagonal() * imu_msg.gyro + (Eigen::Vector3f::Ones() - alpha_lp).asDiagonal() * filtered_gyro;
            imu_msg.gyro_filtered = filtered_gyro;  

            imu_pub.push(imu_msg);            
        }
    }

    void magTask(void *pvParameters)
    {
        int16_t mag_raw[3] = {0};
        int16_t last_mag_raw[3] = {0};
        ImuMagMsg mag_msg;

        const TickType_t delay = pdMS_TO_TICKS(5); // 200 Hz
        TickType_t last_wake_time = xTaskGetTickCount();

        // need to somehow clear the ready bit of the HMC5883L status register to get a data ready signal.
        // currently just polls at 200 hz and does a comparison to see if data has changed.
        while (1){
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                mag.readRawBurst(&mag_raw[0], &mag_raw[1], &mag_raw[2]);
                // only publish if there is new data (poor mans data ready pin)
                if (mag_raw[0] != last_mag_raw[0] || mag_raw[1] != last_mag_raw[1] || mag_raw[2] != last_mag_raw[2]) {
                    mag_msg.timestamp = micros();
                    mag_msg.mag << mag_raw[0] * MAG_LSB_2_uT, 
                                mag_raw[1] * MAG_LSB_2_uT,
                                mag_raw[2] * MAG_LSB_2_uT;

                    // rotate into FRD
                    mag_msg.mag = MAG_TO_BODY_ROT * MAG_TO_FRD_ROT * (mag_msg.mag - mag_bias);
                    mag_pub.push(mag_msg);
                
                    last_mag_raw[0] = mag_raw[0];
                    last_mag_raw[1] = mag_raw[1];
                    last_mag_raw[2] = mag_raw[2];
                };
                xSemaphoreGive(i2c_mutex);
            }
            vTaskDelayUntil(&last_wake_time, delay);
        }
    }
}