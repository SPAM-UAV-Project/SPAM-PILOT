#include "rotor_control.hpp"
#include "sensors/encoder/encoder.hpp"  // for atomic encoder angle

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
        float arming_throttle = 0.10f;
        float max_blade_angle = 0.50f; // virtual angle

        // sysid vars
        float torque_coeff_top = 0.0167; // 187
        float torque_coeff_bot = 0.0155;
        float thrust_coeff_top = 10.9837;
        float thrust_coeff_bot = 9.3460;    
        float top_motor_arm = 0.18f; // meters    
        float amp_cut_in = 0.185f;
        float phase_lag = 30 * M_PI / 180.0f; // 90 degrees phase lag

        Eigen::Vector4f motor_forces = Eigen::Vector4f::Zero(); // f1x, f1y, f1z, f2z
        Eigen::Vector4f body_commands = Eigen::Vector4f::Zero(); // thrust, torque_x, torque_y, torque_z
        Eigen::Matrix4f effectiveness_matrix = Eigen::Matrix4f::Zero();
        // fill effectiveness matrix
        effectiveness_matrix << 
            0.0f, 0.0f, -1.0f, -1.0f,
            0, top_motor_arm, 0.0f, 0.0f, 
            -top_motor_arm, 0.0f, 0.0f, 0.0f,
            0, 0, torque_coeff_top, -torque_coeff_bot;

        Eigen::Matrix4f allocation_matrix = effectiveness_matrix.inverse();
        Eigen::Vector2f blade_xy = Eigen::Vector2f::Zero(); //  blade flapping angle
        Eigen::Vector2f u_motor_sp = Eigen::Vector2f::Zero(); // motor commands
        
        while (true)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // receive data
            thrust_sp_sub_.pull_if_new(thrust_sp_msg_);
            vehicle_state_sub_.pull_if_new(vehicle_state_);

            // this runs at around 1000 hz
            if (torque_sp_sub_.pull_if_new(torque_sp_msg_)){
                // allocate motor commands
                body_commands << thrust_sp_msg_.setpoint, torque_sp_msg_.setpoint.x(), torque_sp_msg_.setpoint.y(), torque_sp_msg_.setpoint.z();
                // map to motor forces
                motor_forces = allocation_matrix * body_commands;
                
                // convert to bx, by using top motor force only
                // dont do anything if motor forces are low
                if (motor_forces(2) > 1) { // 1 N threshold
                    blade_xy.y() = -motor_forces(0) / (motor_forces(2) + 1e-6f); // f1x / f1z  = B_y
                    blade_xy.x() = -motor_forces(1) / (motor_forces(2) + 1e-6f); // f1y / f1z = B_x

                    // for swashplateless rotor control, we need to find an amplitude and a phase lag
                    amplitude = amp_cut_in + sqrt(SQ(blade_xy(0)) + SQ(blade_xy(1)));
                    amplitude = std::min(amplitude, max_blade_angle); // cap amplitude to avoid excessive commands / vibrations
                    phase = atan2(blade_xy(1), blade_xy(0));
                } else {
                    blade_xy << 0.0f, 0.0f;
                    amplitude = 0.0f;
                    phase = 0.0f;
                }

                // u = sqrt(thrust / k)
                u_motor_sp(0) = sqrt(std::max(0.0f, motor_forces(2) / thrust_coeff_top));  // top motor
                u_motor_sp(1) = sqrt(std::max(0.0f, motor_forces(3) / thrust_coeff_bot));  // bottom motor

                // publish for logging
                motor_forces_msg_.timestamp = micros();
                motor_forces_msg_.force_setpoint = motor_forces;
                motor_forces_msg_.actuator_setpoints << blade_xy.x(), blade_xy.y(), u_motor_sp(0), u_motor_sp(1);
                motor_forces_pub_.push(motor_forces_msg_);
            }

            // convert to an oscillatory throttle response
            // we use an atomic variable to avoid jitter from the pub/sub system
            float current_encoder_angle = sensors::encoder::atomic_enc_angle_rad.load(std::memory_order_relaxed);
            
            motor1_output = u_motor_sp(0) + amplitude * cos(current_encoder_angle - phase - phase_lag);
            // motor1_output = u_motor_sp(0);
            // motor 2 just controls yaw and thrust
            motor2_output = u_motor_sp(1);

            // constrain from arming throttle to 1.0 (should rarely clip now)
            motor1_output = std::max(arming_throttle, std::min(1.0f, motor1_output));
            motor2_output = std::max(arming_throttle, std::min(1.0f, motor2_output));

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
        }
    }
    
    void ControlAllocator::sendToDshot(float& throttle_fraction, DShotRMT &motor)
    { 
        if (throttle_fraction <= 0.0f) {
            motor.sendThrottle(0);
            return;
        }
        // ensure throttle_fraction is within [0.0, 1.0]
        throttle_fraction = std::max(0.0f, std::min(1.0f, throttle_fraction));
        // convert to DShot value (48 to 2047 for throttle)
        uint16_t dshot_value = static_cast<uint16_t>(48 + throttle_fraction * (2047 - 48));
        motor.sendThrottle(dshot_value);
    }
}