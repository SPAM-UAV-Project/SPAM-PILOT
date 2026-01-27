/**
 * @file msg_broker.hpp
 * @brief Lock-free pub/sub message broker for inter-task communication
 * 
 * Uses double-buffer seqlock pattern:
 * - Publisher writes to inactive buffer, then swaps
 * - Readers always read from active buffer (never block)
 * - Version counter detects torn reads
 */

#ifndef MSG_BROKER_HPP
#define MSG_BROKER_HPP

#include <Arduino.h>
#include <atomic>

template<typename T>
class Topic { 
public:

    /**
     * @brief Publisher class for publishing messages of type T
     * @note Can define multiple publishers, but only one should be actively publishing at a time
     */
    class Publisher {
    public:
        Publisher() {
            Topic<T>::get_instance(); // ensure topic exists
        }

        /**
         * @brief Publish data to the topic (lock-free)
         * @param[in] data Data to publish
         * @note Safe to call from any task. Not ISR safe.
         */
        void push(const T& data) {
            TopicInstance& topic = Topic<T>::get_instance();
            
            topic.version_.fetch_add(1, std::memory_order_release); // increment version to odd (write in progress)

            uint32_t inactive = 1 - topic.active_.load(std::memory_order_acquire); // write to inactive buffer
            topic.buffers_[inactive] = data;
            topic.active_.store(inactive, std::memory_order_release); // swap inactive buffer to active
            
            topic.version_.fetch_add(1, std::memory_order_release); // increment version to even (write complete)
        }
    };

    /**
     * @brief Subscriber class for subscribing to messages of type T
     * @note Multiple subscribers per topic are supported
     */
    class Subscriber {
    public:
        Subscriber() : last_version_(0) {
            Topic<T>::get_instance();
        }

        /**
         * @brief Pull new data from the topic if available (lock-free)
         * @param[out] data_out Reference to store the pulled data
         * @return true if new data was pulled, false otherwise
         * @note Never blocks. Returns false if no new data or write in progress.
         */
        bool pull_if_new(T& data_out) {
            TopicInstance& topic = Topic<T>::get_instance();
            
            // version check if write in progress
            uint32_t v1 = topic.version_.load(std::memory_order_acquire);
            
            // skip now if write in progress or no new data
            if ((v1 & 1) || v1 == last_version_) {
                return false;
            }
            
            // copy from active buffer
            uint32_t active = topic.active_.load(std::memory_order_acquire);
            T temp = topic.buffers_[active];
            
            // version check to see if write happened during copy. return false if so
            uint32_t v2 = topic.version_.load(std::memory_order_acquire);
            if (v1 != v2) {
                return false;
            }
            
            // return data
            data_out = temp;
            last_version_ = v2;
            return true;
        }

    private:
        uint32_t last_version_;
    };

private:
    struct TopicInstance {
        T buffers_[2];                      // double buffer
        std::atomic<uint32_t> active_{0};   // index of readable buffer (0 or 1)
        std::atomic<uint32_t> version_{0};  // seqlock version (even=stable, odd=writing)
    };
    
    static TopicInstance& get_instance() {
        static TopicInstance instance;
        return instance;
    }
};

#endif // MSG_BROKER_HPP