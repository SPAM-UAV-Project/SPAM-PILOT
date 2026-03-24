/**
 * @file encoder.hpp
 * @brief Header file for optical flow sensor (mtf02p) interface
 */

#ifndef OPTIFLOW_HPP
#define OPTIFLOW_HPP

#include <Arduino.h>

namespace sensors::optiflow
{
    static TaskHandle_t optiflowTaskHandle;

    void initOptiflow();
    void optiflowTask(void *pvParameters);
}

#endif // OPTIFLOW_HPP