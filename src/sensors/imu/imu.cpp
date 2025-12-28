#include "imu.hpp"
#include "MPU6050.h"
#include "pin_defs.hpp"
#include "msgs/ImuHighRateMsg.hpp"

namespace sensors::imu
{
    MPU6050 mpu;
    Topic<ImuHighRateMsg>::Publisher imu_pub;

    void initIMU()
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
        Wire.setClock(400000); // 400kHz 

        mpu.initialize();

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

        // start imu task
        xTaskCreate(imuTask, "IMU Task", 2048, NULL, 3, &imuTaskHandle);

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
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        ImuHighRateMsg imu_msg;

        while (1){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // read imu data
            mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

            imu_msg.timestamp = micros();
            imu_msg.accel << ax_raw / 8192.0f * 9.81f,
                             ay_raw / 8192.0f * 9.81f,
                             az_raw / 8192.0f * 9.81f;
            imu_msg.gyro << gx_raw / 65.5f,
                            gy_raw / 65.5f,
                            gz_raw / 65.5f;

            imu_pub.push(imu_msg);
        }
    }
}