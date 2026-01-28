#include "rotor_control.hpp"


// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

namespace gnc
{
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
        motor1_.begin();
        motor2_.begin();
        motor1_.sendThrottle(0);
        motor2_.sendThrottle(0);

        Serial.println("[Rotor Controller]: Initializing rotor control...");
        for(int i = 0; i < 300; i++) {  // 3 seconds of zero throttle
            delay(10);
        }

        xTaskCreatePinnedToCore(allocatorTaskEntry, "Control Allocator Task", 4096, NULL, 5, &allocator_task_handle_, 0);
    
        // create timer
        Serial.println("[Rotor Controller]: Setting up rotor control timer...");
        rotor_control_timer_ = timerBegin(1000000); // 1 MHz timer
        timerAttachInterrupt(rotor_control_timer_, &onRotorControlTimerEntry);
        timerAlarm(rotor_control_timer_, 500, true, 0); // 2000 Hz alarm, auto-reload
        Serial.println("[Rotor Controller]: Rotor control initialized.");
    }

    void ControlAllocator::allocatorTask(void *pvParameters)
    {
        float amplitude = 0.0f;
        float phase = 0.0f;
        float motor1_output = 0.0f;
        float motor2_output = 0.0f;
        float arming_throttle = 0.10f;
        float max_blade_angle = 0.30f; // radians

        // sysid vars
        float torque_coeff_top = 0.2051;
        float torque_coeff_bot = 0.1455;
        float thrust_coeff_top = 10.9837;
        float thrust_coeff_bot = 9.3460;    
        float top_motor_arm = 0.08f; // meters    
        float amp_cut_in = 0.16f;
        float phase_lag = M_PI / 6.0f; // 30 degrees phase lag

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
            encoder_sub_.pull_if_new(encoder_msg_);
            thrust_sp_sub_.pull_if_new(thrust_sp_msg_);

            // this runs at around 1000 hz
            if (torque_sp_sub_.pull_if_new(torque_sp_msg_)){
                // allocate motor commands
                body_commands << thrust_sp_msg_.setpoint, torque_sp_msg_.setpoint.x(), torque_sp_msg_.setpoint.y(), torque_sp_msg_.setpoint.z();
                // map to motor forces
                motor_forces = allocation_matrix * body_commands;
                // convert to u_top, u_bot, bx, by
                blade_xy.x() = -motor_forces[0] / (motor_forces(3) + 1e-6f); // avoid div by zero
                blade_xy.y() = -motor_forces[1] / (motor_forces(3) + 1e-6f); // avoid div by zero
                // u = sqrt(thrust / k)
                u_motor_sp(0) = sqrt(std::max(0.0f, motor_forces(3) / thrust_coeff_top));
                u_motor_sp(1) = sqrt(std::max(0.0f, motor_forces(3) / thrust_coeff_bot));

                // for swashplateless rotor control, we need to find an amplitude and a phase lag
                amplitude = amp_cut_in + sqrt(SQ(blade_xy(0)) + SQ(blade_xy(1)));
                amplitude = std::min(amplitude, max_blade_angle); // cap amplitude to avoid excessive commands / vibrations
                phase = atan2(blade_xy(1), blade_xy(0));
            }

            // convert to an oscillatory throttle response
            motor1_output = u_motor_sp(0) + amplitude * cos(encoder_msg_.angle_rad - phase - phase_lag);
            // motor 2 just controls yaw and thrust
            motor2_output = u_motor_sp(1);

            sendToDshot(motor1_output, motor1_);
            sendToDshot(motor2_output, motor2_);
        }
    }
    
    void ControlAllocator::sendToDshot(float& throttle_fraction, DShotRMT &motor)
    { 
        if (throttle_fraction <= 0.0f) {
            return;
        }
        // ensure throttle_fraction is within [0.0, 1.0]
        throttle_fraction = std::max(0.0f, std::min(1.0f, throttle_fraction));
        // convert to DShot value (48 to 2047 for throttle)
        uint16_t dshot_value = static_cast<uint16_t>(48 + throttle_fraction * (2047 - 48));
        motor.sendThrottle(dshot_value);
    }
}