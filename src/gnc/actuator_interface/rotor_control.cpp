#include "rotor_control.hpp"
#include "sensors/encoder/encoder.hpp"  // for atomic encoder angle
// #include "timing/task_timing.hpp"

// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

namespace gnc
{
    hw_timer_t* ControlAllocator::rotor_control_timer_ = nullptr;
    ControlAllocator* ControlAllocator::instance_ = nullptr;
    
    // control timer interrupt for precise timing
    void IRAM_ATTR ControlAllocator::onRotorControlTimer() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // notify the rotor control task to run
        vTaskNotifyGiveFromISR(allocator_task_handle_, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    
    void ControlAllocator::initRotor()
    {
        instance_ = this;

        log_queue_ = xQueueCreate(50, sizeof(LogPacket));

        xTaskCreatePinnedToCore(allocatorTaskEntry, "Control Allocator Task", 8192, this, 6, &allocator_task_handle_, 1);
        xTaskCreate(loggingTaskEntry, "Logger", 4096, this, 1, &logging_task_handle_);

        // create timer
        Serial.println("[Rotor Controller]: Setting up rotor control timer...");
        rotor_control_timer_ = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(rotor_control_timer_, &onRotorControlTimerEntry);
        Serial.println("[Rotor Controller]: Rotor control initialized.");
    }

    void ControlAllocator::allocatorTask(void *pvParameters)
    {
        Serial.println("[Rotor Controller]: Allocator task started!");
        
        float amplitude = 0.0f;
        float phase = 0.0f;
        float motor1_output = 0.0f;
        float motor2_output = 0.0f;
        float arming_throttle = 0.08f;
        float max_blade_angle = 0.50f; // virtual angle

        // sysid vars
        constexpr float TORQUE_COEFF_TOP = 0.0155f;
        constexpr float TORQUE_COEFF_BOT = 0.0155f;
        constexpr float THRUST_COEFF_TOP_INV = (1.0f / 10.9837f);
        constexpr float THRUST_COEFF_BOT_INV = (1.0f / 9.3460f);
        constexpr float TOP_MOTOR_ARM = 0.18f;
        constexpr float AMP_CUT_IN = 0.17f;
        constexpr float PHASE_LAG = (30.0f * M_PI / 180.0f);

        Eigen::Vector4f motor_forces = Eigen::Vector4f::Zero(); // f1x, f1y, f1z, f2z
        Eigen::Vector4f body_commands = Eigen::Vector4f::Zero(); // thrust, torque_x, torque_y, torque_z
        Eigen::Matrix4f effectiveness_matrix = Eigen::Matrix4f::Zero();
        // fill effectiveness matrix
        effectiveness_matrix << 
            0.0f, 0.0f, -1.0f, -1.0f,
            0, TOP_MOTOR_ARM, 0.0f, 0.0f, 
            -TOP_MOTOR_ARM, 0.0f, 0.0f, 0.0f,
            0, 0, TORQUE_COEFF_TOP, -TORQUE_COEFF_BOT;

        Eigen::Matrix4f allocation_matrix = effectiveness_matrix.inverse();
        Eigen::Vector2f blade_xy = Eigen::Vector2f::Zero(); //  blade flapping angle
        Eigen::Vector2f u_motor_sp = Eigen::Vector2f::Zero(); // motor commands

        uint32_t count = 0;

        // TaskTiming task_timer("RotorAlloc", 1000); // 1000us budget for 1kHz

        // init motors
        motor1_.begin();
        motor2_.begin();
        motor1_.sendThrottle(0);
        motor2_.sendThrottle(0);

        Serial.println("[Rotor Controller]: Initializing rotor control...");
        for(int i = 0; i < 300; i++) {  // 3 seconds of zero throttle
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // start the timer
        timerAlarm(rotor_control_timer_, 1000, true, 0); // 1000 Hz alarm, auto-reload

        
        while (true)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // task_timer.startCycle();
            count++;

            // receive data
            thrust_sp_sub_.pull_if_new(thrust_sp_msg_);
            vehicle_state_sub_.pull_if_new(vehicle_state_);
            torque_sp_sub_.pull_if_new(torque_sp_msg_);

            // run math at 500hz
            if (count % 2 == 0) {
                // allocate motor commands
                body_commands << thrust_sp_msg_.setpoint, torque_sp_msg_.setpoint.x(), torque_sp_msg_.setpoint.y(), torque_sp_msg_.setpoint.z();
                // map to motor forces
                motor_forces.noalias() = allocation_matrix * body_commands;
                
                // convert to bx, by using top motor force only
                // dont do anything if motor forces are low
                if (motor_forces(2) > 1) { // 1 N threshold
                    blade_xy.y() = -motor_forces(0) / (motor_forces(2) + 1e-6f); // f1x / f1z  = B_y
                    blade_xy.x() = -motor_forces(1) / (motor_forces(2) + 1e-6f); // f1y / f1z = B_x
                    
                    // for swashplateless rotor control, we need to find an amplitude and a phase lag
                    amplitude = AMP_CUT_IN + 2 * (sqrtf(SQ(blade_xy(0)) + SQ(blade_xy(1))));
                    amplitude = std::min(amplitude, max_blade_angle); // cap amplitude to avoid excessive commands / vibrations
                    phase = atan2f(blade_xy(1), blade_xy(0));
                } else {
                    blade_xy.setZero();
                    amplitude = 0.0f;
                    phase = 0.0f;
                }

                // u = sqrt(thrust / k)
                u_motor_sp(0) = sqrtf(std::max(0.0f, motor_forces(2) * THRUST_COEFF_TOP_INV));  // top motor
                u_motor_sp(1) = sqrtf(std::max(0.0f, motor_forces(3) * THRUST_COEFF_BOT_INV));  // bottom motor

                // publish for logging
                LogPacket packet;
                packet.timestamp = micros();
                packet.force_setpoint = motor_forces;
                packet.actuator_sp[0] = blade_xy.x();
                packet.actuator_sp[1] = blade_xy.y();
                packet.actuator_sp[2] = u_motor_sp(0);
                packet.actuator_sp[3] = u_motor_sp(1);
                
                xQueueSend(log_queue_, &packet, 0);
            }

            // convert to an oscillatory throttle response
            // we use an atomic variable to avoid jitter from the pub/sub system
            float current_encoder_angle = sensors::encoder::atomic_enc_angle_rad.load(std::memory_order_relaxed);

            motor1_output = u_motor_sp(0) + amplitude * cosf(current_encoder_angle - phase - PHASE_LAG);
            // motor1_output = u_motor_sp(0);
            // motor 2 just controls yaw and thrust
            motor2_output = u_motor_sp(1);

            motor1_output = std::clamp(motor1_output, arming_throttle, 1.0f);
            motor2_output = std::clamp(motor2_output, arming_throttle, 1.0f);

            // disarm if state not explicitly ARMED
            bool is_armed = (vehicle_state_.system_state == SystemState::ARMED ||
                            vehicle_state_.system_state == SystemState::ARMED_FLYING);
            
            if (!is_armed) {
                // Disarmed or stale state - force zero throttle
                motor1_output = 0.0f;
                motor2_output = 0.0f;
            } 

            sendToDshot(motor1_output, motor1_);
            sendToDshot(motor2_output, motor2_);

            // task_timer.endCycle();
            // if (task_timer.getCycleCount() % 1000 == 0) {
            //     task_timer.printStats();
            // }
        }
    }
    
    void ControlAllocator::sendToDshot(float& throttle_fraction, DShotRMT &motor)
    { 
        if (throttle_fraction <= 0.0f) {
            motor.sendThrottle(0);
            return;
        }
        // ensure throttle_fraction is within [0.0, 1.0]
        throttle_fraction = std::clamp(throttle_fraction, 0.0f, 1.0f);
        // convert to DShot value (48 to 2047 for throttle)
        uint16_t dshot_value = static_cast<uint16_t>(48 + throttle_fraction * (2047 - 48));
        motor.sendThrottle(dshot_value);
    }

    void ControlAllocator::loggingTask(void* pvParameters) 
    {
        LogPacket packet;
        while (true) {
            if (xQueueReceive(log_queue_, &packet, portMAX_DELAY)) {
                motor_forces_msg_.timestamp = packet.timestamp;
                motor_forces_msg_.force_setpoint = packet.force_setpoint;
                motor_forces_msg_.actuator_setpoints << packet.actuator_sp[0], packet.actuator_sp[1], 
                                                        packet.actuator_sp[2], packet.actuator_sp[3];
                motor_forces_pub_.push(motor_forces_msg_); 
            }
        }
    }
}

