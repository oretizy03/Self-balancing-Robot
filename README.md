# Self balancing Robot

Firmware for a two-wheel self-balancing robot based on an **ESP32**. The project focuses on accurate motor control, encoder-based speed measurement, motor calibration, balancing calibration, and **PID-based stabilization**.

The firmware is developed as a sequence of test and calibration stages rather than treating the balancing algorithm as a single program. This makes it possible to verify the motors, encoders, sensors, and control parameters independently before combining them into the final self-balancing system.

The project includes:

1. Encoder reading and verification
2. Motor speed calibration and speed matching
3. Balancing/sensor calibration
4. PID-based self-balancing control
5. Bluetooth-based manual driving
6. Motor braking and controlled motion testing



## Project Goals

The main goals of this project are:
1.  Build a two-wheel self-balancing robot around an ESP32.
2. Measure motor rotation using quadrature or pulse-based wheel encoders.
3. Determine the actual operating speed of each motor at different PWM values.
4. Compensate for differences between the left and right motors.
5. Establish a repeatable calibration process before implementing balancing.
6. Use an inertial measurement unit (IMU) to estimate the robot's tilt.
7. Implement PID feedback control to maintain the robot near its vertical position.
8. Provide Bluetooth control for forward, reverse, left, and right movement.
9. Keep the robot's center of gravity fixed relative to the wheel axle so that the balancing controller operates from a consistent mechanical configuration.



## PROJECT STATUS: 65% COMPLETE
