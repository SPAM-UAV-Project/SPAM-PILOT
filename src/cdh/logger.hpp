#ifndef TS_LOGGER_HPP
#define TS_LOGGER_HPP

#include <Arduino.h>

namespace CDH {

class Logger {

public: 
    Logger();
    ~Logger() = default;

    void begin();
    void start();
    void stop();

    static void loggerTaskEntry(void* instance) {
        static_cast<Logger*>(instance)->loggerTask();
    }

    TaskHandle_t loggerTaskHandle = nullptr;

private:
    void loggerTask();
    bool currently_logging_ = false;
};

} // namespace CDH

#endif // TS_LOGGER_HPP
