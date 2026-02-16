#ifndef SD_LOGGER_HPP
#define SD_LOGGER_HPP

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <atomic>
#include <algorithm>
#include "pin_defs.hpp"

// Message types and broker
#include "msg_broker.hpp"
#include "msgs/ImuHighRateMsg.hpp"
#include "msgs/EkfStatesMsg.hpp"
#include "msgs/MotorForcesMsg.hpp"
#include "msgs/AttitudeSetpointMsg.hpp"
#include "msgs/RateSetpointMsg.hpp"
#include "msgs/TorqueSetpointMsg.hpp"
#include "msgs/EncoderMsg.hpp"
#include "msgs/ImuIntegrated.hpp"
#include "msgs/EkfInnovations.hpp"
#include "msgs/ImuMagMsg.hpp"
#include "msgs/AngAccelMsg.hpp"

namespace cdh {


// ============================================================================
// Configuration
// ============================================================================
constexpr size_t SD_BUFFER_SIZE = 65536;        // 64KB ring buffer
constexpr size_t SD_FLUSH_THRESHOLD = 16384;    // Flush at 16KB (larger chunks = better efficiency)
constexpr uint32_t SD_LOG_PERIOD_MS = 2;        // 500Hz polling (2ms)
constexpr uint32_t SD_WRITE_PERIOD_MS = 100;    // Writer checks at 10Hz
constexpr uint32_t SD_SPI_FREQ = 20000000;      // 20MHz SPI

// ============================================================================
// Binary Log Format
// ============================================================================

// Message type IDs for binary format
enum class LogMsgType : uint8_t {
    IMU = 0,
    EKF = 1,
    MOTOR_FORCES = 2,
    ATT_SP = 3,
    RATE_SP = 4,
    TORQUE_SP = 5,
    ENCODER = 6,
    IMU_INTEGRATED = 7,
    EKF_INNOVATIONS = 8,
    MAG = 9,
    ANG_ACCEL = 10
};

// Binary log entry header (4 bytes)
struct __attribute__((packed)) LogHeader {
    uint8_t msg_type;       // LogMsgType
    uint8_t payload_size;   // Size of payload in bytes
    uint16_t reserved;      // Alignment padding
};

// Binary payload structures (packed for minimal size)
struct __attribute__((packed)) LogImu {
    uint64_t timestamp;
    float accel[3];
    float gyro[3];
    float accel_filtered[3];
    float gyro_filtered[3];
};  // 56 bytes

struct __attribute__((packed)) LogEkf {
    uint64_t timestamp;
    float position[3];
    float velocity[3];
    float attitude[4];  // w,x,y,z
};  // 48 bytes

struct __attribute__((packed)) LogMotorForces {
    uint64_t timestamp;
    float forces[4];
    float actuators[4];
};  // 40 bytes

struct __attribute__((packed)) LogAttSp {
    uint64_t timestamp;
    float q[4];
    float yaw_ff;
};  // 28 bytes

struct __attribute__((packed)) LogRateSp {
    uint64_t timestamp;
    float rate[3];
};  // 20 bytes

struct __attribute__((packed)) LogTorqueSp {
    uint64_t timestamp;
    float torque[3];
    float delayed_torque[3];
    float indi_increment[3];
};  // 44 bytes

struct __attribute__((packed)) LogEncoder {
    uint64_t timestamp;
    float angle_rad;
    float angular_velocity_rad_s;
};  // 16 bytes

struct __attribute__((packed)) LogImuIntegrated {
    uint64_t timestamp;
    float delta_vel[3];
    float delta_angle[3];
    float delta_vel_dt;
    float delta_angle_dt;
};  // 40 bytes

struct __attribute__((packed)) LogEkfInnovations {
    uint64_t timestamp;
    float mag_innov[3];
    float mag_innov_cov[3]; // diagonal covariance
    float gravity_innov[3];
    float gravity_innov_cov[3]; // diagonal covariance
};  // 56 bytes

struct __attribute__((packed)) LogMag {
    uint64_t timestamp;
    float mag[3];
};  // 20 bytes

struct __attribute__((packed)) LogAngAccel {
    uint64_t timestamp;
    float ang_accel_raw[3];
    float ang_accel_filtered[3];
};  // 32 bytes

// File header
struct __attribute__((packed)) LogFileHeader {
    uint32_t magic;         // 0x4D415053 = "SPAM"
    uint16_t version;       // Format version
    uint16_t header_size;   // Size of this header
    uint64_t start_time_us; // micros() at session start
};

/**
 * @brief High-performance binary SD logger with autonomous topic subscription
 * 
 * Subscribes to all relevant topics and logs new data automatically.
 * Uses binary format for speed and small file size.
 * 
 * Usage:
 *   SdLogger logger;
 *   logger.init();           // Initialize SD and start task
 *   logger.startSession();   // Start logging (call on ARM)
 *   logger.stopSession();    // Stop logging (call on DISARM)
 */
class SdLogger {
public:
    SdLogger() = default;
    ~SdLogger() = default;

    /**
     * @brief Initialize SD card and start logger task
     * @return true if SD initialized successfully
     */
    bool init();

    /**
     * @brief Start logging session (opens new file)
     * @return true if file created
     */
    bool startSession();

    /**
     * @brief Stop logging session (flush and close)
     * @note Blocks until buffer fully written
     */
    void stopSession();

    /**
     * @brief Check if logging is active
     */
    bool isLogging() const { return logging_active_.load(std::memory_order_acquire); }

    /**
     * @brief Get dropped message count
     */
    uint32_t getDropCount() const { return drop_count_.load(std::memory_order_relaxed); }

    /**
     * @brief Get current buffer usage in bytes
     */
    size_t getBufferUsage() const;

private:
    // Ring buffer
    uint8_t buffer_[SD_BUFFER_SIZE];
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};

    // State
    std::atomic<bool> logging_active_{false};
    std::atomic<uint32_t> drop_count_{0};
    
    File log_file_;
    bool sd_initialized_ = false;
    uint16_t current_file_num_ = 0;
    uint64_t session_start_us_ = 0;

    // Topic subscribers (autonomous logging)
    Topic<ImuHighRateMsg>::Subscriber imu_sub_;
    Topic<EkfStatesMsg>::Subscriber ekf_sub_;
    Topic<MotorForcesMsg>::Subscriber motor_sub_;
    Topic<AttitudeSetpointMsg>::Subscriber att_sp_sub_;
    Topic<RateSetpointMsg>::Subscriber rate_sp_sub_;
    Topic<TorqueSetpointMsg>::Subscriber torque_sp_sub_;
    Topic<EncoderMsg>::Subscriber encoder_sub_;
    Topic<ImuIntegratedMsg>::Subscriber imu_integrated_sub_;
    Topic<EkfInnovationsMsg>::Subscriber ekf_innovations_sub_;
    Topic<ImuMagMsg>::Subscriber mag_sub_;
    Topic<AngAccelMsg>::Subscriber ang_accel_sub_;

    // Message buffers for subscribers
    ImuHighRateMsg imu_msg_;
    EkfStatesMsg ekf_msg_;
    MotorForcesMsg motor_msg_;
    AttitudeSetpointMsg att_sp_msg_;
    RateSetpointMsg rate_sp_msg_;
    TorqueSetpointMsg torque_sp_msg_;
    EncoderMsg encoder_msg_;
    ImuIntegratedMsg imu_integrated_msg_;
    EkfInnovationsMsg ekf_innovations_msg_;
    ImuMagMsg mag_msg_;
    AngAccelMsg ang_accel_msg_;

    TaskHandle_t logger_task_handle_ = nullptr;
    TaskHandle_t writer_task_handle_ = nullptr;

    // Buffer operations
    bool writeToBuffer(const void* data, size_t len);
    bool writeLogEntry(const LogHeader& hdr, const void* payload, size_t payload_len);
    size_t availableForWrite() const;
    size_t availableForRead() const;
    void drainBuffer();
    uint16_t findNextFileNumber();

    // Tasks
    void loggerTask();      // 500Hz - polls subscribers
    void writerTask();      // 20Hz - drains buffer to SD
    
    static void loggerTaskEntry(void* instance) {
        static_cast<SdLogger*>(instance)->loggerTask();
    }
    static void writerTaskEntry(void* instance) {
        static_cast<SdLogger*>(instance)->writerTask();
    }

    // Log helpers
    bool logImu();
    bool logEkf();
    bool logMotorForces();
    void logAttSp();
    void logRateSp();
    void logTorqueSp();
    void logEncoder();
    void logImuIntegrated();
    void logEkfInnovations();
    void logMag();
    void logAngAccel();
};

} // namespace cdh

#endif // SD_LOGGER_HPP
