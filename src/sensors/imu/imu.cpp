/**
 * @file imu.cpp
 * @brief implementation for IMU interface and tasks
 */

#include "imu.hpp"
#include "hmc5883l.hpp"
#include "MPU6050.h"
#include "pin_defs.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/ImuIntegrated.hpp"
#include "msgs/ImuMagMsg.hpp"
#include "filter/butter_lp.hpp"
#include "filter/notch.hpp"
#include "integrator/integrator.hpp"

#include "timing/task_timing.hpp"

namespace sensors::imu
{
    TwoWire imuI2C = TwoWire(1);  // use I2C bus 1 for IMU

    MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &imuI2C);
    HMC5883L mag;
    Topic<ImuHighRateMsg>::Publisher imu_pub;
    Topic<ImuIntegratedMsg>::Publisher imu_int_pub;
    Topic<ImuMagMsg>::Publisher mag_pub;
    Integrator accel_integrator;
    Integrator gyro_integrator;

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

        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
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
        xTaskCreate(imuTask, "IMU Task", 8192, NULL, 4, &imuTaskHandle);

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
        xTaskCreatePinnedToCore(magTask, "Mag Task", 8192, NULL, 2, NULL, 1);

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

        constexpr float f_s = 1000.0f; // sample freq
        constexpr uint32_t ekf_dt_thres_us = 3900; // threshold to reset integrator

        // setup notch filter
        Eigen::Vector3f notched_gyro = Eigen::Vector3f::Zero();
        Eigen::Vector3f notched_accel = Eigen::Vector3f::Zero();
        float gyro_notch_freq = 55.0f;
        float gyro_notch_bw = 10.0f;
        float accel_notch_freq = 55.0f;
        float accel_notch_bw = 10.0f;
        NotchFilt gyro_notch_filter_;
        NotchFilt accel_notch_filter_;
        gyro_notch_filter_.setup(gyro_notch_freq, gyro_notch_bw, f_s);
        accel_notch_filter_.setup(accel_notch_freq, accel_notch_bw, f_s);

        // setup low pass filter
        float gyro_f_c = 35.0f;
        float accel_f_c = 35.0f;
        ButterLowPassFilt gyro_lp_filter_;
        ButterLowPassFilt accel_lp_filter_;
        gyro_lp_filter_.setup(gyro_f_c, f_s);
        accel_lp_filter_.setup(accel_f_c, f_s);

        // constexpr for optimization
        const Eigen::Matrix3f IMU_ROT = IMU_TO_BODY_ROT * IMU_TO_FRD_ROT;
        constexpr float accel_lsb_to_m_s2 = 9.81f / 2048.0f; // for FS = +/-4g
        constexpr float gyro_lsb_to_rad_s = DEG_TO_RAD / 65.5f;

        // timing for integration and pubs
        uint32_t last_timestamp_us = micros();

        // TaskTiming task_timer("IMU Task", 1000);

        while (1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // task_timer.startCycle();

            uint32_t current_timestamp_us = micros();
            uint32_t dt_us = current_timestamp_us - last_timestamp_us;
            last_timestamp_us = current_timestamp_us;

            // read imu data
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                mpu.getMotion6(&imu_raw[0], &imu_raw[1], &imu_raw[2], &imu_raw[3], &imu_raw[4], &imu_raw[5]);
                xSemaphoreGive(i2c_mutex);
            }

            imu_msg.timestamp = current_timestamp_us;
            imu_msg.accel << imu_raw[0] * accel_lsb_to_m_s2,
                             imu_raw[1] * accel_lsb_to_m_s2,
                             imu_raw[2] * accel_lsb_to_m_s2;
            imu_msg.gyro << imu_raw[3] * gyro_lsb_to_rad_s,
                            imu_raw[4] * gyro_lsb_to_rad_s,
                            imu_raw[5] * gyro_lsb_to_rad_s;
            // rotate into FRD
            imu_msg.gyro = IMU_ROT * imu_msg.gyro;
            imu_msg.accel = IMU_ROT * imu_msg.accel;

            // filter accel and gyro with notch, then low pass -> these go to controllers only
            gyro_notch_filter_.apply3d(imu_msg.gyro.data(), notched_gyro.data());
            accel_notch_filter_.apply3d(imu_msg.accel.data(), notched_accel.data());

            gyro_lp_filter_.apply3d(notched_gyro.data(), imu_msg.gyro_filtered.data());
            accel_lp_filter_.apply3d(notched_accel.data(), imu_msg.accel_filtered.data());

            imu_pub.push(imu_msg);    

            // integrate at 250hz for ekf prediction
            accel_integrator.integrate3d(notched_accel, dt_us); // dt in us
            gyro_integrator.integrate3d(notched_gyro, dt_us);

            if (accel_integrator.isReady(ekf_dt_thres_us) && gyro_integrator.isReady(ekf_dt_thres_us)) {
                ImuIntegratedMsg imu_int_msg;
                imu_int_msg.timestamp = current_timestamp_us;

                accel_integrator.getAndReset3d(imu_int_msg.delta_vel, imu_int_msg.delta_vel_dt);
                gyro_integrator.getAndReset3d(imu_int_msg.delta_angle, imu_int_msg.delta_angle_dt);

                imu_int_pub.push(imu_int_msg);
            }

            // task_timer.endCycle();

            // // if 1 second has passed, print
            // if (task_timer.getCycleCount() % 1000 == 0) {
            //     task_timer.printStats();
            // }
        }
    }

    void magTask(void *pvParameters)
    {
        int16_t mag_raw[3] = {0};
        int16_t last_mag_raw[3] = {0};
        ImuMagMsg mag_msg;
        const Eigen::Matrix3f MAG_ROT = MAG_TO_BODY_ROT * MAG_TO_FRD_ROT;

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
                    mag_msg.mag = MAG_ROT * (mag_msg.mag - mag_bias);
                    mag_pub.push(mag_msg);
                
                    last_mag_raw[0] = mag_raw[0];
                    last_mag_raw[1] = mag_raw[1];
                    last_mag_raw[2] = mag_raw[2];

                    // for mag cal
                    // Serial.printf("%lf\t%lf\t%lf\n", mag_msg.mag[0], mag_msg.mag[1], mag_msg.mag[2]);
                };
                xSemaphoreGive(i2c_mutex);
            }
            vTaskDelayUntil(&last_wake_time, delay);
        }
    }
}