#ifndef SRV_BROKER_HPP
#define SRV_BROKER_HPP

/**
 * @file srv_broker.hpp
 * @brief Simple RPC for synchronous updates
 * 
 * To use: define a service as a struct, with request and response sub-structs. see spam_msgs/srv
 * This is a blocking call request-response mechanism.
 */

#include <Arduino.h>

template<typename ServiceType>
class Service {
    using Req = typename ServiceType::Request;
    using Res = typename ServiceType::Response;
    using Callback = bool (*)(void*, const Req&, Res&);

    struct ServiceBus {
        Callback callback = nullptr;
        void* context = nullptr;
        SemaphoreHandle_t mutex;

        ServiceBus() {
            mutex = xSemaphoreCreateMutex();
        }
    };

    static ServiceBus& get_bus() {
        static ServiceBus bus;
        return bus;
    }

public:
    /**
     * @brief Advertising class for hosting the service
     */
    class Server {
    public:
        template <typename T, bool (T::*Method)(const Req&, Res&)>
        void advertise(T* instance) {
            ServiceBus& bus = Service<ServiceType>::get_bus();
            if (xSemaphoreTake(bus.mutex, portMAX_DELAY) == pdTRUE) {
                bus.context = instance;
                bus.callback = [](void* obj, const Req& r, Res& rs) -> bool {
                    return (static_cast<T*>(obj)->*Method)(r, rs);
                };
                xSemaphoreGive(bus.mutex);
            }
        }
    };

    /**
     * @brief Calling class for invoking the service
     */
    class Client {
    public:
        bool call(const Req& req, Res& res, TickType_t timeout = pdMS_TO_TICKS(100)) {
            ServiceBus& bus = Service<ServiceType>::get_bus();
            if (xSemaphoreTake(bus.mutex, timeout) == pdTRUE) {
                bool success = false;
                if (bus.callback && bus.context) {
                    success = bus.callback(bus.context, req, res);
                }
                xSemaphoreGive(bus.mutex);
                return success;
            }
            return false;
        }
    };
};

#endif // SRV_BROKER_HPP