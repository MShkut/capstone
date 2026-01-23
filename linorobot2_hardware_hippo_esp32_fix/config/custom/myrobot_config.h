#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

// Robot configuration
#define NUM_MOTORS 2
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_2_IN_MOTOR_DRIVER  // VNH5019 uses 2-pin direction control

// Motor driver pins - VNH5019
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 25    // Left motor PWM
  #define MOTOR1_IN_A 26   // Left INA
  #define MOTOR1_IN_B 27   // Left INB

  #define MOTOR2_PWM 4    // Right motor PWM
  #define MOTOR2_IN_A 5   // Right INA
  #define MOTOR2_IN_B 18   // Right INB

  #define PWM_MAX 1023     // ESP32 max PWM value
  #define PWM_MIN -PWM_MAX
#endif

// Encoder pins
#define MOTOR1_ENCODER_A 22  // Left encoder A
#define MOTOR1_ENCODER_B 23  // Left encoder B

#define MOTOR2_ENCODER_A 19   // Right encoder A
#define MOTOR2_ENCODER_B 21   // Right encoder B

// Invert encoders if direction is wrong (test and adjust)
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false

// Motor characteristics (MEASURE THESE FOR YOUR MOTORS)
#define MOTOR_MAX_RPM 200          // Max RPM of your motors
#define MAX_RPM_RATIO 0.85         // Safety margin (85% of max)
#define MOTOR_OPERATING_VOLTAGE 12 // Your motor voltage
#define MOTOR_POWER_MAX_VOLTAGE 12 // VNH5019 input voltage
#define COUNTS_PER_REV1 1320       // Encoder ticks per revolution (LEFT - CALIBRATE THIS)
#define COUNTS_PER_REV2 1320       // Encoder ticks per revolution (RIGHT - CALIBRATE THIS)

// Robot dimensions (MEASURE YOUR ROBOT)
#define WHEEL_DIAMETER 0.065       // Wheel diameter in meters
#define TRACK_WIDTH 0.25           // Distance between wheels in meters

// PID coefficients (START WITH THESE, TUNE LATER)
#define K_P 0.6
#define K_I 0.3
#define K_D 0.5

// Micro-ROS settings
#define USE_SERIAL_TRANSPORT true  // USB serial (change to USE_WIFI_TRANSPORT for wireless)
#define BAUDRATE 115200            // Serial baud rate

// PWM settings for ESP32 and VNH5019
#define PWM_FREQUENCY 20000      // 20 kHz typical
#define PWM_BITS 10              // 10-bit resolution
#define MOTOR1_INV false         // invert if motor spins backward
#define MOTOR2_INV false

// Robot dimensions
#define LR_WHEELS_DISTANCE TRACK_WIDTH

// Status LED
#define LED_PIN 2                // built-in ESP32 LED or any free GPIO

// IMU (optional)
#define USE_IMU false
// #define IMU imu              // only if USE_IMU true

#endif
