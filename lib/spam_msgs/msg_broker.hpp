/**
 * @file msg_broker.hpp
 * @brief pub/sub message broker for inter-task communication
 */

#ifndef MSG_BROKER_HPP
#define MSG_BROKER_HPP

#include <Arduino.h>

template<typename T>
class Topic { 
public:

    /**
     * @brief Publisher class for publishing messages of type T
     * @note Only one publisher per topic is allowed
     */
    class Publisher {
    public:
        Publisher() {
            Topic<T>::get_topic_instance(); // create the topic
        }
        /**
         * @brief Publish data to the topic 
         * @note This blocks until the mutex is acquired
         * @param[in] data Data to publish
         * @warning This is not ISR safe
         */
        void push(const T& data) {
            TopicInstance& top = Topic<T>::get_topic_instance();
            if (xSemaphoreTake(top.mutex_, portMAX_DELAY) == pdTRUE) {
                top.data_ = data;
                top.version_++;
                xSemaphoreGive(top.mutex_);
            }
        }
    };

    /**
     * @brief Subscriber class for subscribing to messages of type T
     */
    class Subscriber {
    public:
        Subscriber() : last_version_(0) {
            Topic<T>::get_topic_instance(); // create the topic
        }
        /**
         * @brief Pulls new data from the topic if available, uses versioning to check new data
         * @note This blocks until the mutex is acquired
         * @param[out] data_out Reference to store the pulled data
         * @return True if new data was pulled, false otherwise
         * @warning This is not ISR safe
         */
        bool pull_if_new(T& data_out) {
            TopicInstance& top = Topic<T>::get_topic_instance();
            if (top.version_ != last_version_) {
                if (xSemaphoreTake(top.mutex_, portMAX_DELAY) == pdTRUE) {
                    data_out = top.data_;
                    last_version_ = top.version_;
                    xSemaphoreGive(top.mutex_);
                    return true;
                }
            }
            return false;
        }
    private:
        uint32_t last_version_;
    };

private:
    /**
     * @brief Internal structure to hold topic data.
     */
    struct TopicInstance{
        T data_;
        volatile uint32_t version_ = 0;
        SemaphoreHandle_t mutex_ = xSemaphoreCreateMutex();
    };
    
    /**
     * @brief Get the topic instance object
     * @return Reference to the topic instance. If it doesn't exist, it will be created
     */
    static TopicInstance& get_topic_instance() {
        static TopicInstance top;
        return top;
    }
};


#endif // MSG_BROKER_HPP