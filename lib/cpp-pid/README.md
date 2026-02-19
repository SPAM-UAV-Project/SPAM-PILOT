# cpp-pid

Practical header-only C++ library for deploying a PID on embedded systems / general control systems projects.

Features:
* Different implementations of the PID controller:
    * PID with the derivative on measurement ([Derivative Kick](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/))
   * Standard PID implementation with derivative on error
* Integral windup clamping
* Output clamping
* Low-pass filter on the derivative term
* Pre-multiplication of the time step with the integral and derivative gains

As used in: \
[A cascaded TVC rocket project](https://github.com/CAPTR-Project/CAPTR-V1-AVI) \
[A cascaded retractable drone slung payload project](https://github.com/UTAT-UAS/ARCHIVE_hardware_integration/tree/main/packages/ros-payload-control)
[A custom flight controller for a coaxial drone](https://github.com/SPAM-UAV-Project/SPAM-PILOT)

## Implementations

### The PID algorithm

The PID controller computes the control output based on the error between a desired setpoint and the measured process variable. The general formula for this discrete PID controller is given by:

\[ u_n = K \left( K_p e_n + K_i T_s \sum_{j=0}^{n} e_j + \frac{K_d}{T_s} (e_n - e_{n-1}) \right) \]

Where:
* `K` is the overall system gain, is 1 in parallel form, and can be used to scale output
* `K_p, K_i, K_d` are the regular pid gains

This implementation is available in `pid.hpp`.

### Derivative on Measurement
This implementation calculates the derivative term based on the measurement (process variable) rather than the error. This approach helps to reduce the "derivative kick" effect that can occur when there are sudden changes in the setpoint due to a large error derivative spike at those moments.

This implementation is available in `pid_dom.hpp`.

