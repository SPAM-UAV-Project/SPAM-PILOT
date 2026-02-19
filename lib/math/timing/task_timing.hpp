#ifndef TASK_TIMING_HPP
#define TASK_TIMING_HPP

#include <Arduino.h>
#include <atomic>

/**
 * @brief Lightweight task timing profiler with windowed stats
 * 
 * Tracks both cumulative (total) and windowed (per-print-interval) statistics.
 * printStats() shows window stats and resets them for next interval.
 */
class TaskTiming {
public:
    TaskTiming(const char* name, uint32_t budget_us = 0)
        : name_(name), budget_us_(budget_us) {}
    
    // Call at start of work
    inline void startCycle() {
        cycle_start_us_ = micros();
        
        // Track period (time since last cycle start)
        if (last_cycle_start_us_ > 0) {
            uint32_t period = cycle_start_us_ - last_cycle_start_us_;
            // Cumulative
            period_sum_us_ += period;
            if (period > period_max_us_) period_max_us_ = period;
            if (period < period_min_us_) period_min_us_ = period;
            // Window
            win_period_sum_us_ += period;
            if (period > win_period_max_us_) win_period_max_us_ = period;
            if (period < win_period_min_us_) win_period_min_us_ = period;
        }
        last_cycle_start_us_ = cycle_start_us_;
    }
    
    // Call at end of work (before delay)
    inline void endCycle() {
        uint32_t exec_us = micros() - cycle_start_us_;
        
        // Cumulative stats
        exec_sum_us_ += exec_us;
        if (exec_us > exec_max_us_) exec_max_us_ = exec_us;
        if (exec_us < exec_min_us_) exec_min_us_ = exec_us;
        
        // Window stats
        win_exec_sum_us_ += exec_us;
        if (exec_us > win_exec_max_us_) win_exec_max_us_ = exec_us;
        if (exec_us < win_exec_min_us_) win_exec_min_us_ = exec_us;
        
        if (budget_us_ > 0 && exec_us > budget_us_) {
            overrun_count_++;
            win_overrun_count_++;
        }
        
        cycle_count_++;
        win_cycle_count_++;
    }
    
    // Get cumulative stats
    uint32_t getCycleCount() const { return cycle_count_; }
    uint32_t getOverrunCount() const { return overrun_count_; }
    uint32_t getExecMaxUs() const { return exec_max_us_; }
    uint32_t getExecMinUs() const { return exec_min_us_; }
    uint32_t getExecAvgUs() const { 
        return cycle_count_ > 0 ? exec_sum_us_ / cycle_count_ : 0; 
    }
    uint32_t getPeriodMaxUs() const { return period_max_us_; }
    uint32_t getPeriodMinUs() const { return period_min_us_; }
    uint32_t getPeriodAvgUs() const { 
        return cycle_count_ > 1 ? period_sum_us_ / (cycle_count_ - 1) : 0; 
    }
    
    // Get window stats
    uint32_t getWinExecAvgUs() const {
        return win_cycle_count_ > 0 ? win_exec_sum_us_ / win_cycle_count_ : 0;
    }
    uint32_t getWinPeriodAvgUs() const {
        return win_cycle_count_ > 1 ? win_period_sum_us_ / (win_cycle_count_ - 1) : 0;
    }
    
    // Reset all stats (cumulative + window)
    void reset() {
        cycle_count_ = 0;
        overrun_count_ = 0;
        exec_sum_us_ = 0;
        exec_max_us_ = 0;
        exec_min_us_ = UINT32_MAX;
        period_sum_us_ = 0;
        period_max_us_ = 0;
        period_min_us_ = UINT32_MAX;
        last_cycle_start_us_ = 0;
        resetWindow();
    }
    
    // Reset only window stats
    void resetWindow() {
        win_cycle_count_ = 0;
        win_overrun_count_ = 0;
        win_exec_sum_us_ = 0;
        win_exec_max_us_ = 0;
        win_exec_min_us_ = UINT32_MAX;
        win_period_sum_us_ = 0;
        win_period_max_us_ = 0;
        win_period_min_us_ = UINT32_MAX;
    }
    
    // Print WINDOW stats (current interval only) and reset window
    void printStats() {
        Serial.printf("[%s] win=%lu/%lu exec(us): avg=%lu min=%lu max=%lu period(us): avg=%lu min=%lu max=%lu | total=%lu\n",
            name_,
            win_cycle_count_,
            win_overrun_count_,
            getWinExecAvgUs(),
            win_exec_min_us_ == UINT32_MAX ? 0 : win_exec_min_us_,
            win_exec_max_us_,
            getWinPeriodAvgUs(),
            win_period_min_us_ == UINT32_MAX ? 0 : win_period_min_us_,
            win_period_max_us_,
            cycle_count_
        );
        resetWindow();
    }
    
    // Compact single-line stats for logging
    void getStatsString(char* buf, size_t len) const {
        snprintf(buf, len, "%s:%lu/%lu/%lu/%lu",
            name_,
            getWinExecAvgUs(),
            win_exec_max_us_,
            win_overrun_count_,
            win_cycle_count_
        );
    }

private:
    const char* name_;
    uint32_t budget_us_;
    
    // Timing state
    uint32_t cycle_start_us_ = 0;
    uint32_t last_cycle_start_us_ = 0;
    
    // Cumulative stats (since boot/reset)
    uint32_t cycle_count_ = 0;
    uint32_t overrun_count_ = 0;
    uint64_t exec_sum_us_ = 0;
    uint32_t exec_max_us_ = 0;
    uint32_t exec_min_us_ = UINT32_MAX;
    uint64_t period_sum_us_ = 0;
    uint32_t period_max_us_ = 0;
    uint32_t period_min_us_ = UINT32_MAX;
    
    // Window stats (since last printStats)
    uint32_t win_cycle_count_ = 0;
    uint32_t win_overrun_count_ = 0;
    uint64_t win_exec_sum_us_ = 0;
    uint32_t win_exec_max_us_ = 0;
    uint32_t win_exec_min_us_ = UINT32_MAX;
    uint64_t win_period_sum_us_ = 0;
    uint32_t win_period_max_us_ = 0;
    uint32_t win_period_min_us_ = UINT32_MAX;
};

#endif // TASK_TIMING_HPP
