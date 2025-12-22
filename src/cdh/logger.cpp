#include "logger.hpp"

namespace CDH {

Logger::Logger(){}

void Logger::start() {
    if (!currently_logging_) {
        currently_logging_ = true;
        Serial.println("time_ms, torque_x, torque_z, thrust, torque_x_sp, thrust_sp");

    }
}

void Logger::stop() {
    if (currently_logging_) {
        currently_logging_ = false;
        Serial.println("[Logger] Logging stopped");
    }
}

void Logger::begin() {
    xTaskCreate(loggerTaskEntry, "LoggerTask", 4096, this, 1, &loggerTaskHandle);
    Serial.println("[Logger] Logger task started.");
}

void Logger::loggerTask() {

    while (true) {
        // placeholder for logger functionality 

        vTaskDelay(pdMS_TO_TICKS(10)); // log at 100 Hz
    }
}

} // namespace CDH