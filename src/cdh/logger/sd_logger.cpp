#include "sd_logger.hpp"
// #include "timing/task_timing.hpp"

namespace cdh {

// ============================================================================
// Initialization
// ============================================================================

bool SdLogger::init() {
    gpio_reset_pin((static_cast<gpio_num_t>(PIN_SD_CS)));
    gpio_reset_pin((static_cast<gpio_num_t>(PIN_SD_SCK)));
    gpio_reset_pin((static_cast<gpio_num_t>(PIN_SD_MOSI)));
    gpio_reset_pin((static_cast<gpio_num_t>(PIN_SD_MISO)));
    pinMode(PIN_SD_CS, OUTPUT);
    pinMode(PIN_SD_SCK, OUTPUT);
    pinMode(PIN_SD_MOSI, OUTPUT);
    pinMode(PIN_SD_MISO, INPUT);
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    
    if (!SD.begin(PIN_SD_CS, SPI, SD_SPI_FREQ)) {
        Serial.println("[SdLogger] SD card init failed!");
        return false;
    }
    
    sd_initialized_ = true;
    current_file_num_ = findNextFileNumber();
    Serial.println("[SdLogger] SD initialized.");
    

    xTaskCreatePinnedToCore(loggerTaskEntry, "SdLogger", 4096, this, 2, &logger_task_handle_, 1);
    xTaskCreatePinnedToCore(writerTaskEntry, "SdWriter", 8192, this, 2, &writer_task_handle_, 1);
    
    Serial.println("[SdLogger] Tasks started on Core 1.");
    return true;
}

uint16_t SdLogger::findNextFileNumber() {
    uint16_t num = 1;
    char filename[20];
    
    while (num < 9999) {
        snprintf(filename, sizeof(filename), "/LOG_%04u.bin", num);
        if (!SD.exists(filename)) break;
        num++;
    }
    return num;
}

// ============================================================================
// Session Control
// ============================================================================

bool SdLogger::startSession() {
    if (!sd_initialized_ || logging_active_.load(std::memory_order_acquire)) {
        return false;
    }
    
    char filename[20];
    snprintf(filename, sizeof(filename), "/LOG_%04u.bin", current_file_num_);
    
    log_file_ = SD.open(filename, FILE_WRITE);
    if (!log_file_) {
        Serial.printf("[SdLogger] Failed to create: %s\n", filename);
        return false;
    }
    
    // Write file header
    session_start_us_ = micros();
    LogFileHeader header = {
        .magic = 0x4D415053,  // "SPAM"
        .version = 1,
        .header_size = sizeof(LogFileHeader),
        .start_time_us = session_start_us_
    };
    log_file_.write((uint8_t*)&header, sizeof(header));
    log_file_.flush();
    
    // Reset buffer
    head_.store(0, std::memory_order_release);
    tail_.store(0, std::memory_order_release);
    drop_count_.store(0, std::memory_order_release);
    
    logging_active_.store(true, std::memory_order_release);
    Serial.printf("[SdLogger] Started: %s\n", filename);
    current_file_num_++;
    
    return true;
}

void SdLogger::stopSession() {
    if (!logging_active_.load(std::memory_order_acquire)) return;
    
    // Stop logging new data
    logging_active_.store(false, std::memory_order_release);
    
    // Give writer task a moment to finish current write
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Drain remaining buffer
    size_t remaining = availableForRead();
    Serial.printf("[SdLogger] Stopping... Buffer remaining: %u bytes\n", remaining);
    drainBuffer();
    
    // Close file
    if (log_file_) {
        log_file_.flush();
        size_t file_size = log_file_.size();
        log_file_.close();
        Serial.printf("[SdLogger] File closed. Size: %u bytes\n", file_size);
    }
    
    Serial.printf("[SdLogger] Stopped. Drops: %lu\n", drop_count_.load());
}

// ============================================================================
// Ring Buffer
// ============================================================================

size_t SdLogger::availableForWrite() const {
    size_t h = head_.load(std::memory_order_acquire);
    size_t t = tail_.load(std::memory_order_acquire);
    return (h >= t) ? (SD_BUFFER_SIZE - (h - t) - 1) : (t - h - 1);
}

size_t SdLogger::availableForRead() const {
    size_t h = head_.load(std::memory_order_acquire);
    size_t t = tail_.load(std::memory_order_acquire);
    return (h >= t) ? (h - t) : (SD_BUFFER_SIZE - t + h);
}

size_t SdLogger::getBufferUsage() const {
    return availableForRead();
}

bool SdLogger::writeToBuffer(const void* data, size_t len) {
    if (len > availableForWrite()) {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    const uint8_t* src = (const uint8_t*)data;
    size_t h = head_.load(std::memory_order_relaxed);
    
    for (size_t i = 0; i < len; i++) {
        buffer_[h] = src[i];
        h = (h + 1) % SD_BUFFER_SIZE;
    }
    
    head_.store(h, std::memory_order_release);
    return true;
}

bool SdLogger::writeLogEntry(const LogHeader& hdr, const void* payload, size_t payload_len) {
    // Check space for BOTH header and payload atomically
    size_t total_len = sizeof(LogHeader) + payload_len;
    if (total_len > availableForWrite()) {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    
    // Write header + payload as single contiguous operation
    size_t h = head_.load(std::memory_order_relaxed);
    
    // Write header
    const uint8_t* hdr_bytes = reinterpret_cast<const uint8_t*>(&hdr);
    for (size_t i = 0; i < sizeof(LogHeader); i++) {
        buffer_[h] = hdr_bytes[i];
        h = (h + 1) % SD_BUFFER_SIZE;
    }
    
    // Write payload
    const uint8_t* payload_bytes = reinterpret_cast<const uint8_t*>(payload);
    for (size_t i = 0; i < payload_len; i++) {
        buffer_[h] = payload_bytes[i];
        h = (h + 1) % SD_BUFFER_SIZE;
    }
    
    // Single atomic update - writer task sees complete message or nothing
    head_.store(h, std::memory_order_release);
    return true;
}

void SdLogger::drainBuffer() {
    if (!log_file_) return;
    
    size_t readable = availableForRead();
    if (readable == 0) return;
    
    constexpr size_t CHUNK_SIZE = 4096;
    uint8_t chunk[CHUNK_SIZE];
    
    size_t t = tail_.load(std::memory_order_relaxed);
    size_t written = 0;
    
    while (written < readable) {
        size_t to_write = std::min(CHUNK_SIZE, readable - written);
        
        // Copy to chunk buffer first (safe to do since we own the tail)
        size_t current_tail = t;
        for (size_t i = 0; i < to_write; i++) {
            chunk[i] = buffer_[current_tail];
            current_tail = (current_tail + 1) % SD_BUFFER_SIZE;
        }
        
        // Write chunk to SD
        size_t bytes_written = log_file_.write(chunk, to_write);
        
        if (bytes_written != to_write) {
            Serial.printf("[SdLogger] Write failed! Expected %u, wrote %u\n", to_write, bytes_written);
            
            // Critical failure - try to recover by reopening file
            if (bytes_written == 0) {
                Serial.println("[SdLogger] Write failed 0 bytes. Reopening...");
                // vTaskDelay(pdMS_TO_TICKS(50)); // Delay removed as requested
                
                log_file_.close();
                
                // Try to reopen in APPEND mode
                char filename[20];
                snprintf(filename, sizeof(filename), "/LOG_%04u.bin", current_file_num_ - 1);
                log_file_ = SD.open(filename, FILE_APPEND);
                
                if (log_file_) {
                    Serial.println("[SdLogger] Reopened file successfully");
                } else {
                    Serial.println("[SdLogger] Failed to reopen file!");
                    // If reopening fails, stop logging to prevent infinite error loop
                    logging_active_.store(false, std::memory_order_release);
                }
                break;
            }
        }
        
        written += bytes_written;
        
        // Advance tail by amount actually written
        // If partial write occurred, we only advance by that amount
        // Recalculate t based on original t + bytes_written
        t = (t + bytes_written) % SD_BUFFER_SIZE;
        tail_.store(t, std::memory_order_release);
    }
    
    log_file_.flush();
}

// ============================================================================
// Logger Task (500Hz) - Polls subscribers and writes to buffer
// ============================================================================

void SdLogger::loggerTask() {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t log_count = 0;
    uint32_t imu_count = 0, ekf_count = 0, motor_count = 0;  // Debug counters
    
    while (true) {
        if (logging_active_.load(std::memory_order_acquire)) {
            // Poll all subscribers and log new data
            if (logImu()) imu_count++;
            if (logEkf()) ekf_count++;
            if (logMotorForces()) motor_count++;
            logAttSp();
            logRateSp();
            logTorqueSp();
            logEncoder();
            logImuIntegrated();
            logEkfInnovations();
            
            // Debug: print stats every second (500 cycles at 500Hz)
            log_count++;
            if (log_count % 500 == 0) {
                // Serial.printf("[SdLogger] Polls: %lu, IMU: %lu, EKF: %lu, Motor: %lu, Buf: %u\n", 
                //    log_count, imu_count, ekf_count, motor_count, availableForRead());
            }
        } else {
            log_count = 0;
            imu_count = 0;
            ekf_count = 0;
            motor_count = 0;
        }
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SD_LOG_PERIOD_MS));
    }
}

// ============================================================================
// Writer Task (20Hz) - Drains buffer to SD card
// ============================================================================

void SdLogger::writerTask() {
    TickType_t last_wake = xTaskGetTickCount();
    // TaskTiming task_timer("SdWriter", 50000); // 50000us budget for 20Hz
    
    while (true) {
        // task_timer.startCycle();
        // Only write when logging and threshold reached
        if (logging_active_.load(std::memory_order_acquire) &&
            availableForRead() >= SD_FLUSH_THRESHOLD) {
            drainBuffer();
        }
        // task_timer.endCycle();
        // if (task_timer.getCycleCount() % 20 == 0) {
        //     task_timer.printStats();
        // }
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SD_WRITE_PERIOD_MS));
    }
}

// ============================================================================
// Log Helpers - Convert messages to binary and write to buffer
// ============================================================================

bool SdLogger::logImu() {
    if (!imu_sub_.pull_if_new(imu_msg_)) return false;
    
    LogHeader hdr = { 
        .msg_type = static_cast<uint8_t>(LogMsgType::IMU),
        .payload_size = sizeof(LogImu),
        .reserved = 0
    };
    
    LogImu payload;
    payload.timestamp = imu_msg_.timestamp;
    payload.accel[0] = imu_msg_.accel.x();
    payload.accel[1] = imu_msg_.accel.y();
    payload.accel[2] = imu_msg_.accel.z();
    payload.gyro[0] = imu_msg_.gyro.x();
    payload.gyro[1] = imu_msg_.gyro.y();
    payload.gyro[2] = imu_msg_.gyro.z();
    payload.accel_filtered[0] = imu_msg_.accel_filtered.x();
    payload.accel_filtered[1] = imu_msg_.accel_filtered.y();
    payload.accel_filtered[2] = imu_msg_.accel_filtered.z();
    payload.gyro_filtered[0] = imu_msg_.gyro_filtered.x();
    payload.gyro_filtered[1] = imu_msg_.gyro_filtered.y();
    payload.gyro_filtered[2] = imu_msg_.gyro_filtered.z();
    
    writeLogEntry(hdr, &payload, sizeof(payload));
    return true;
}

bool SdLogger::logEkf() {
    if (!ekf_sub_.pull_if_new(ekf_msg_)) return false;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::EKF),
        .payload_size = sizeof(LogEkf),
        .reserved = 0
    };
    
    LogEkf payload;
    payload.timestamp = ekf_msg_.timestamp;
    payload.position[0] = ekf_msg_.position.x();
    payload.position[1] = ekf_msg_.position.y();
    payload.position[2] = ekf_msg_.position.z();
    payload.velocity[0] = ekf_msg_.velocity.x();
    payload.velocity[1] = ekf_msg_.velocity.y();
    payload.velocity[2] = ekf_msg_.velocity.z();
    payload.attitude[0] = ekf_msg_.attitude.w();
    payload.attitude[1] = ekf_msg_.attitude.x();
    payload.attitude[2] = ekf_msg_.attitude.y();
    payload.attitude[3] = ekf_msg_.attitude.z();
    
    writeLogEntry(hdr, &payload, sizeof(payload));
    return true;
}

bool SdLogger::logMotorForces() {
    if (!motor_sub_.pull_if_new(motor_msg_)) return false;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::MOTOR_FORCES),
        .payload_size = sizeof(LogMotorForces),
        .reserved = 0
    };
    
    LogMotorForces payload;
    payload.timestamp = motor_msg_.timestamp;
    payload.forces[0] = motor_msg_.force_setpoint.x();
    payload.forces[1] = motor_msg_.force_setpoint.y();
    payload.forces[2] = motor_msg_.force_setpoint.z();
    payload.forces[3] = motor_msg_.force_setpoint.w();
    payload.actuators[0] = motor_msg_.actuator_setpoints.x();
    payload.actuators[1] = motor_msg_.actuator_setpoints.y();
    payload.actuators[2] = motor_msg_.actuator_setpoints.z();
    payload.actuators[3] = motor_msg_.actuator_setpoints.w();
    
    writeLogEntry(hdr, &payload, sizeof(payload));
    return true;
}

void SdLogger::logAttSp() {
    if (!att_sp_sub_.pull_if_new(att_sp_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::ATT_SP),
        .payload_size = sizeof(LogAttSp),
        .reserved = 0
    };
    
    LogAttSp payload;
    payload.timestamp = att_sp_msg_.timestamp;
    payload.q[0] = att_sp_msg_.q_sp.w();
    payload.q[1] = att_sp_msg_.q_sp.x();
    payload.q[2] = att_sp_msg_.q_sp.y();
    payload.q[3] = att_sp_msg_.q_sp.z();
    payload.yaw_ff = att_sp_msg_.yaw_sp_ff_rate;
    
    writeLogEntry(hdr, &payload, sizeof(payload));
}

void SdLogger::logRateSp() {
    if (!rate_sp_sub_.pull_if_new(rate_sp_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::RATE_SP),
        .payload_size = sizeof(LogRateSp),
        .reserved = 0
    };
    
    LogRateSp payload;
    payload.timestamp = rate_sp_msg_.timestamp;
    payload.rate[0] = rate_sp_msg_.setpoint.x();
    payload.rate[1] = rate_sp_msg_.setpoint.y();
    payload.rate[2] = rate_sp_msg_.setpoint.z();
    
    writeLogEntry(hdr, &payload, sizeof(payload));
}

void SdLogger::logTorqueSp() {
    if (!torque_sp_sub_.pull_if_new(torque_sp_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::TORQUE_SP),
        .payload_size = sizeof(LogTorqueSp),
        .reserved = 0
    };
    
    LogTorqueSp payload;
    payload.timestamp = torque_sp_msg_.timestamp;
    payload.torque[0] = torque_sp_msg_.setpoint.x();
    payload.torque[1] = torque_sp_msg_.setpoint.y();
    payload.torque[2] = torque_sp_msg_.setpoint.z();
    
    writeLogEntry(hdr, &payload, sizeof(payload));
}

void SdLogger::logEncoder() {
    if (!encoder_sub_.pull_if_new(encoder_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::ENCODER),
        .payload_size = sizeof(LogEncoder),
        .reserved = 0
    };
    
    LogEncoder payload;
    payload.timestamp = encoder_msg_.timestamp;
    payload.angle_rad = encoder_msg_.angle_rad;
    payload.angular_velocity_rad_s = encoder_msg_.angular_velocity_rad_s;
    
    writeLogEntry(hdr, &payload, sizeof(payload));
}

void SdLogger::logImuIntegrated() {
    if (!imu_integrated_sub_.pull_if_new(imu_integrated_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::IMU_INTEGRATED),
        .payload_size = sizeof(LogImuIntegrated),
        .reserved = 0
    };
    
    LogImuIntegrated payload;
    payload.timestamp = imu_integrated_msg_.timestamp;
    payload.delta_vel[0] = imu_integrated_msg_.delta_vel.x();
    payload.delta_vel[1] = imu_integrated_msg_.delta_vel.y();
    payload.delta_vel[2] = imu_integrated_msg_.delta_vel.z();
    payload.delta_angle[0] = imu_integrated_msg_.delta_angle.x();
    payload.delta_angle[1] = imu_integrated_msg_.delta_angle.y();
    payload.delta_angle[2] = imu_integrated_msg_.delta_angle.z();
    payload.delta_vel_dt = imu_integrated_msg_.delta_vel_dt;
    payload.delta_angle_dt = imu_integrated_msg_.delta_angle_dt;
    
    writeLogEntry(hdr, &payload, sizeof(payload));
}

void SdLogger::logEkfInnovations() {
    if (!ekf_innovations_sub_.pull_if_new(ekf_innovations_msg_)) return;
    
    LogHeader hdr = {
        .msg_type = static_cast<uint8_t>(LogMsgType::EKF_INNOVATIONS),
        .payload_size = sizeof(LogEkfInnovations),
        .reserved = 0
    };
    
    LogEkfInnovations payload;
    payload.timestamp = ekf_innovations_msg_.timestamp;
    payload.mag_innov[0] = ekf_innovations_msg_.mag_innov.x();
    payload.mag_innov[1] = ekf_innovations_msg_.mag_innov.y();
    payload.mag_innov[2] = ekf_innovations_msg_.mag_innov.z();
    payload.mag_innov_cov[0] = ekf_innovations_msg_.mag_innov_cov(0);
    payload.mag_innov_cov[1] = ekf_innovations_msg_.mag_innov_cov(1);
    payload.mag_innov_cov[2] = ekf_innovations_msg_.mag_innov_cov(2);

    payload.gravity_innov[0] = ekf_innovations_msg_.gravity_innov.x();
    payload.gravity_innov[1] = ekf_innovations_msg_.gravity_innov.y();
    payload.gravity_innov[2] = ekf_innovations_msg_.gravity_innov.z();
    payload.gravity_innov_cov[0] = ekf_innovations_msg_.gravity_innov_cov(0);
    payload.gravity_innov_cov[1] = ekf_innovations_msg_.gravity_innov_cov(1);
    payload.gravity_innov_cov[2] = ekf_innovations_msg_.gravity_innov_cov(2);

    writeLogEntry(hdr, &payload, sizeof(payload));
}

} // namespace cdh
