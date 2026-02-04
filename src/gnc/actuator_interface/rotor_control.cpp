#include "rotor_control.hpp"
#include "sensors/encoder/encoder.hpp"  // for atomic encoder angle

// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

#include <xtensa/hal.h>
#include <xtensa/config/core-isa.h>
#include <xtensa/config/core-matmap.h>

namespace gnc
{
    hw_timer_t* ControlAllocator::rotor_control_timer_ = nullptr;
    ControlAllocator* ControlAllocator::instance_ = nullptr;
    
    // control timer interrupt for precise timing
    void IRAM_ATTR ControlAllocator::onRotorControlTimer() {
        if (!instance_) return;

        uint32_t cp0_regs[XCHAL_CP0_SA_SIZE / 4] __attribute__((aligned(16)));
        uint32_t cp_state = xthal_get_cpenable();
        if (cp_state & 1) { // Check if CP0 (FPU) is enabled
            xthal_save_cp0(cp0_regs);
        } else {
            xthal_set_cpenable(1); // Enable FPU if it wasn't
        }

        float motor1_output = 0.0f;

        if (instance_->isr_armed_) {
            // convert to an oscillatory throttle response
            // we use an atomic variable and also this ISR to avoid jitter from the pub/sub system
            float current_encoder_angle = sensors::encoder::atomic_enc_angle_rad.load(std::memory_order_relaxed);
            float motor1_output = instance_->isr_u_motor1_sp_ + isr_amp_ * cosf(current_encoder_angle - isr_phase_ - 0.52359877f); // 30 deg phase lag compensation

            motor1_output = std::clamp(motor1_output, 0.08f, 1.0f);
        } else {
            motor1_output = 0.0f;
        }

        
        if(cp_state) {
            // Restore FPU registers
            xthal_restore_cp0(cp0_regs);
        } else {
            // turn it back off
            xthal_set_cpenable(0);
        }

        instance_->sendToDshot(motor1_output, instance_->motor1_);

        
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
        
        motor1_.begin();
        motor2_.begin();
        motor1_.sendThrottle(0);
        motor2_.sendThrottle(0);

        Serial.println("[Rotor Controller]: Initializing rotor control...");
        for(int i = 0; i < 300; i++) {  // 3 seconds of zero throttle
            delay(10);
        }

        xTaskCreate(allocatorTaskEntry, "Control Allocator Task", 8192, this, 3, &allocator_task_handle_);
    
        // create timer
        Serial.println("[Rotor Controller]: Setting up rotor control timer...");
        rotor_control_timer_ = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(rotor_control_timer_, &onRotorControlTimerEntry);
        timerAlarm(rotor_control_timer_, 1000, true, 0); // 1000 Hz alarm, auto-reload
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
        constexpr float AMP_CUT_IN = 0.18f;
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
        uint32_t calls = 0;
        
        while (true)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            calls++;

            if ((calls % 2) == 0) {
                // receive data
                thrust_sp_sub_.pull_if_new(thrust_sp_msg_);
                vehicle_state_sub_.pull_if_new(vehicle_state_);

                // this runs at around 1000 hz
                if (torque_sp_sub_.pull_if_new(torque_sp_msg_)){
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
                        amplitude = AMP_CUT_IN + sqrtf(SQ(blade_xy(0)) + SQ(blade_xy(1)));
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
                }

                // disarm if state not explicitly ARMED
                bool is_armed = (vehicle_state_.system_state == SystemState::ARMED ||
                                vehicle_state_.system_state == SystemState::ARMED_FLYING);

                // motor1_output = u_motor_sp(0);
                // motor 2 just controls yaw and thrust
                // update isr variables at 500 Hz
                isr_amp_ = amplitude;
                isr_phase_ = phase;
                isr_u_motor1_sp_ = u_motor_sp(0);
                isr_armed_ = is_armed;
                motor2_output = u_motor_sp(1);
                motor2_output = std::clamp(motor2_output, arming_throttle, 1.0f);
                
                if (!is_armed) {
                    // force 0 throttle
                    motor2_output = 0.0f;
                } 
                
                // publish for logging
                motor_forces_msg_.timestamp = micros();
                motor_forces_msg_.force_setpoint = motor_forces;
                motor_forces_msg_.actuator_setpoints << blade_xy.x(), blade_xy.y(), u_motor_sp(0), u_motor_sp(1);
                motor_forces_pub_.push(motor_forces_msg_);

                // motor 1 has been sent through ISR for precise timing
                sendToDshot(motor2_output, motor2_);
            }
            
        }
    }
    
    void IRAM_ATTR ControlAllocator::sendToDshot(float& throttle_fraction, DShotRMT &motor)
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
}