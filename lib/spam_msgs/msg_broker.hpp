/**
 * @file msg_broker.hpp
 * @author Tommy (Ziyang) Zhang
 * @brief pub/sub message broker for inter-task communication
 * @date 2025-12-27
 */

#ifndef SPAM_DDS_HPP
#define SPAM_DDS_HPP

#include <Arduino.h>

template<typename T>
class Topic { 
public:

    class Publisher {
    public:
        Publisher() {
            Topic<T>::get_topic_instance(); // create the topic
        }
        void push(const T& data) {
            TopicInstance& top = Topic<T>::get_topic_instance();
            if (xSemaphoreTake(top.mutex_, portMAX_DELAY) == pdTRUE) {
                top.data_ = data;
                top.version_++;
                xSemaphoreGive(top.mutex_);
            }
        }
    };

    class Subscriber {
    public:
        Subscriber() : last_version_(0) {
            Topic<T>::get_topic_instance(); // create the topic
        }
    
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
    struct TopicInstance{
        T data_;
        volatile uint32_t version_ = 0;
        SemaphoreHandle_t mutex_ = xSemaphoreCreateMutex();
    }

    // create a single static message instance when Topic is instantiated
    static TopicInstance& get_topic_instance() {
        static TopicInstance top;
        return top;
    }
};


#endif // SPAM_DDS_HPP