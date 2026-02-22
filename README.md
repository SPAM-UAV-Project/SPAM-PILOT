# SPAM-PILOT

Flight stack for SPAM-UAV - 2 actuator control of 6 degrees of freedom. \
The meat (SPAM meat) and potatoes of the entire project

## Development Details

### Attitude Estimator

Uses an error-state extended Kalman filter formulation ([ESKF](https://arxiv.org/pdf/1711.02508)) to estimate pos, vel, quaternion attitude, gyro and accel biases, where the attitude is formulated in an angular error (more valid for linearization). Currently, the implementation only has attitude estimation since I wanted to focus on fast development of the attitude controller, which could be tested indoors.

The estimator runs at 250Hz, and receives delta_vel and delta_ang measurements from the IMU.

Measurement updates include:
- Accelerometer gravity measurement to correct tilt, gated with a magnitude to ensure we don't fuse while accelerating over a certain point.
- Magnetometer heading measurement to correct yaw, with an innovation gate of 1 std since the place I was testing in had extremely bad magnetic properties (Rebar on the floor).

### Control Scheme

Rotor control uses a very similar implementation as this [paper](https://www.modlabupenn.org/wp-content/uploads/2018/05/paulos_emulating_ICRA_2018.pdf). 

A cascaded attitude and rate PI law is initially chosen as the controller, since, after doing testing, the noise is way too high for a derivative term, even after heavy filtering.
Currently testing INDI (incremental non-linear dynamic inversion) on rate control due to its robustness properties from Simulink Monte Carlo sims (check out the `feature/indi branch`), but faced high noise due to vibrations.

### Firmware Architecture

Uses C++ and FreeRTOS for task management, running on an ESP32-S3 microcontroller.

Inter-task communication used a simple pub/sub architecture with an input and output buffer and checks to ensure that data was fully written. This was the implementation that produced the lowest overhead compared to taking mutexes, while allowing multiple subscribers, which I had trouble with when experimenting with FreeRTOS queues. 
Messages are defined in `lib/spam_msgs`.

Main components consist of:
- State machine (for rc and state flow logic of the thing)
- GNC stuff (for flying the thing)
- Sensor driver stuff (for interfacing with the thing)
- Command and Data handling (for communicating with and data-logging of the thing)
