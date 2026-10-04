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



# System Overview

The robot is divided into several functional sections:
ESP32 (Motor PWM / DIR, Encoder Inputs, IMU / I2C, Button / Buzzer, Bluetooth) > Motor Driver > (Left Motor + encoder) + (Right Motor + encoder) > IMU

The firmware is developed in stages:
Hardware Assembly > Encoder Reading Test > Motor Calibration > Balancing Calibration > PID Stabilization > Bluetooth Driving > Integrated Robot Firmware

Each stage is intended to verify the previous stage before moving forward.



# Hardware / Components

The main hardware used by the robot includes:

1. ESP32 development board
2. Two geared DC motors with wheels
3. Dual-channel motor driver - TB6612FNG
4. Inertial Measurement Unit (IMU) for tilt/angle measurement - MPU6050
5. Robot chassis
6. Two encoders - LM393
7. Battery/power source
8. Buzzer
9. Tactile push button
10. Connecting wires
11. Breadboard or prototyping board
12. Resistors required for input pull-up and other signal conditioning - 10k ohms
13. Mechanical mounting hardware

**Note:** The exact motor driver, encoder type, IMU model, battery configuration, and other hardware should match the components used in the corresponding firmware files.

# Pin Configuration

The following pin assignments are currently used by the firmware.

## Motor Driver

### Motor A — LEFT MOTOR
ESP32 Pin
GPIO 25  PWMA — Motor A PWM
GPIO 26  AIN1 — Motor A direction
GPIO 27  AIN2 — Motor A direction

### Motor B — RIGHT MOTOR
ESP32 Pin
GPIO 14  PWMB — Motor B PWM
GPIO 4  BIN1 — Motor B direction
GPIO 13  BIN2 — Motor B direction

### Motor Driver Standby
ESP32 Pin
GPIO 33  STBY — Motor driver standby
The motor driver must be taken out of standby before the motors can operate.

# Encoder Connections

The two motor encoders are connected to dedicated ESP32 interrupt-capable GPIOs.

### Encoder
Left / Motor A Encoder  GPIO 18
Right / Motor B Encoder  GPIO 19

### Encoder signals

The encoder outputs are used by the firmware to determine wheel rotation and calculate motor speed.
Depending on the encoder hardware, the outputs may provide:
* A single pulse signal, or
* Two phase-shifted signals (quadrature encoding)
The encoder test firmware should be used first to verify that the ESP32 is receiving the expected pulses.


# User Interface Connections

## Buzzer
ESP32 GPIO 23 - Buzzer - GND
The buzzer is used for audible feedback during calibration, startup, or other firmware events.

## Calibration / Control Switch
3.3 V > 10 kΩ > GPIO 35 > Push Button > GND

GPIO 35 is an **input-only ESP32 pin**, so the button input uses an **external pull-up resistor**.
The input is therefore: HIGH = Button not pressed, LOW  = Button pressed.

# I2C Connections

The I2C bus uses the standard ESP32 I2C pins assigned by this project:

## ESP32 Pin 
GPIO 21  SDA
GPIO 22  SCL
3.3V  VCC
GND  GND
The IMU and any other I2C peripherals should share the same SDA and SCL bus, provided their I2C addresses do not conflict.

# Mechanical Configuration

The mechanical design is important because a self-balancing robot does not behave like a conventional two-wheel vehicle. The robot is designed around a **fixed center of gravity relative to the wheel axle**. The position of the battery, controller, motor driver, and other components therefore affects the balancing characteristics.

The following should remain consistent after calibration:
1. Wheel position
2. Motor mounting
3. Battery position
4. IMU position
5. Chassis geometry
6. Overall center of gravity

Changing the mass distribution after PID tuning may change the behavior of the robot and require recalibration.
The IMU should also be mounted securely so that it does not move independently of the chassis.

# Firmware Development Process

The firmware is intentionally separated into individual stages.

## 1. Encoder Reading Test

**File:** `encoder_test.ino`
The first stage verifies that both motor encoders are functioning correctly. The program reads encoder pulses and reports the measured counts through the serial monitor. It's purpose is to verify:
1. Encoder wiring
2. GPIO configuration
3. Interrupt operation
4. Encoder direction, where applicable
5. Pulse/count consistency
6. Left/right encoder identification

Before performing motor calibration, confirm that rotating each wheel produces the expected encoder readings.

### Expected procedure

1. Upload the encoder test firmware.
2. Open the Serial Monitor.
3. Rotate the left wheel 10 times.
4. Confirm that the left encoder count changes.
5. Rotate the right wheel 10 times.
6. Confirm that the right encoder count changes.
7. Check that there are no unexpected counts while the wheel is stationary.

If the encoder readings are incorrect, calibration should not be performed until the problem is resolved.

# 2. Motor Calibration

**File:** `Motor_Calibration.ino`

The two motors will rarely rotate at exactly the same speed when given the same PWM command. Differences may be caused by:
1. Motor manufacturing tolerances
2. Gearbox differences
3. Wheel diameter differences
4. Mechanical friction
5. Chassis loading
6. Battery voltage
7. Motor-driver losses

The motor calibration stage measures the actual speed of each motor using its encoder.

### Calibration objective

------

### Calibration concept

------

The calibration firmware can be used to determine the preferred operating PWM for each motor.

### Important

Motor calibration should be performed with the robot mechanically stable and the testing conditions kept consistent. Avoid changing these between calibration runs whenever possible:
1. Battery voltage
2. Wheel configuration
3. Motor mounting
4. Payload
5. Mechanical load


# 3. Balancing Calibration

**File:** `Balancing_Calibration.ino`

After the motors have been characterized, the next stage is to calibrate the balancing system.This stage establishes the sensor and control reference used by the balancing firmware.The robot must determine what sensor reading corresponds to its intended upright position.

### Main objectives
1. Initialize the IMU.
2. Determine the sensor orientation.
3. Establish the upright reference angle.
4. Verify the direction of tilt measurement.
5. Confirm that the sensor readings are stable.
6. Determine whether filtering is required.
7. Confirm that motor correction direction is correct.

### Upright reference
The balancing controller requires a target angle:
- Target angle = Upright position
- The control system continuously compares the measured angle with the target.
- Angle error = Target angle - Measured angle
  
This error becomes the basis for motor correction.

### Safety
Motor direction must be verified before allowing the robot to balance. When the robot tilts forward, the wheels must move in the direction necessary to move the base underneath the center of gravity. If the correction direction is reversed, the robot will accelerate away from its balanced position rather than correcting the tilt.


# 4. PID Balancing

**File:** `pid.ino`

The PID firmware combines the sensor feedback and calibrated motor control to stabilize the robot. The controller continuously measures the robot's tilt and calculates the motor command required to reduce the error. The basic PID equation is:

u(t) = Kp e(t) + Ki ∫e(t)dt + Kd de(t)/dt

where:
 e(t) = angle error
 Kp = proportional gain
 Ki = integral gain
 Kd = derivative gain
 u(t) = motor control output

## Proportional Term
The proportional component reacts to the current error.

P = Kp × error

A larger tilt produces a larger correction.


## Integral Term

The integral component accumulates error over time.

I = Ki × accumulated error

It can help compensate for persistent small errors, although excessive integral action can cause instability or integral wind-up.


## Derivative Term

The derivative component responds to how quickly the error is changing.

D = Kd × rate of change of error

This helps damp rapid movements and can reduce oscillation.

# Balancing Control Loop

The general control sequence is: IMU > PID > Calculate motor output > Apply motor compensation > Left Motor + Right Motor > Robot changes position > IMU > Feedback .This loop runs continuously while the robot is enabled.

# Motor Speed Matching and PID
Motor calibration and balancing control are related but serve different purposes.

### Motor calibration
Determines how the motors behave: PWM → Actual motor speed

### PID balancing
Determines how the robot should react to its tilt: Tilt error → Required motor correction

Combining both allows the balancing controller to operate with motors that are more closely matched. Without speed matching, identical control commands can produce different wheel speeds, causing unwanted rotational motion.

# 5. Bluetooth Driving

**File:** `bluetooth_drive.ino`

The Bluetooth firmware provides manual control of the robot. Depending on the implemented command protocol, Bluetooth commands can be mapped to actions such as: Forward, Backward, Left, Right, Stop, Brake. This firmware is primarily intended for testing the drivetrain and confirming that the motor-control system behaves correctly.

### Typical control path
Bluetooth command > ESP32 > Motor control function > PWM + Direction > Motor Driver > Left / Right Motors
Bluetooth driving is useful for verifying:
1. Motor direction
2. Left/right motor identification
3. Turning direction
4. Braking
5. PWM response
6. General drivetrain operation



# Recommended Firmware Upload Order

The firmware should be uploaded and tested in the following order.

## Stage 1 — Hardware Assembly

Complete all electrical and mechanical connections.
Verify:
1. ESP32 power
2. Motor driver connections
3. Encoder connections
4. IMU connections
5. Switch connection
6. Buzzer connection
7. Common ground between components
8. 
Do not begin balancing tests until the wiring has been checked.


## Stage 2 — Encoder Test

Upload the encoder reading test.
Confirm that:
LEFT wheel  → LEFT encoder
RIGHT wheel → RIGHT encoder

both produce correct readings after ten turns.


## Stage 3 — Motor Calibration

Upload the motor calibration firmware, measure the motor response and determine suitable PWM values for the left and right motors, save the resulting calibration values, these values should be transferred into the main motor-control/balancing firmware.


## Stage 4 — Balancing Calibration

Upload the balancing calibration firmware.
Use it to:
1. Check IMU orientation
2. Establish the upright position
3. Verify tilt direction
4. Verify correction direction
5. Confirm stable sensor readings


## Stage 5 — PID Firmware

Upload the PID balancing firmware. Start with conservative PID parameters and tune the controller gradually.
A typical tuning process is: > Adjust Kp > Observe response > Adjust Kd > Observe damping > Adjust Ki if required.
The exact tuning procedure depends on the robot's mass, motor characteristics, center of gravity, wheel size, sensor filtering, and control-loop timing.


## Stage 6 — Bluetooth Driving

Upload the Bluetooth driving firmware when manual drivetrain control is required.
This can be used independently of the balancing firmware for testing the robot's movement.


# Safety Considerations

Self-balancing robots can move unexpectedly during testing.
For initial tests:
1. Keep the robot raised off the ground.
2. Secure the robot before testing motors at high PWM.
3. Keep fingers away from the wheels.
4. Verify motor direction before enabling PID control.
5. Confirm the emergency stop/brake function.
6. Use a battery and power system capable of supplying the motor current.
7. Do not make major mechanical changes after PID tuning without expecting recalibration.

The first balancing tests should be performed with a person ready to physically catch or disable the robot.
integration.

# Future Improvements

Possible future improvements include:
1. More advanced sensor filtering
2. Automatic PID parameter tuning
3. Improved encoder-based velocity control
4. Battery voltage monitoring
5. Low-battery protection
6. Data logging
7. Real-time telemetry
8. Wireless PID parameter adjustment9
9. Improved Bluetooth command handling
10. More sophisticated balancing algorithms
11. Automatic startup calibration
12. Fall detection and motor shutdown
13. Performance comparison between different PID configurations
feedback, motor calibration, balancing calibration, PID stabilization, and Bluetooth control stages.
