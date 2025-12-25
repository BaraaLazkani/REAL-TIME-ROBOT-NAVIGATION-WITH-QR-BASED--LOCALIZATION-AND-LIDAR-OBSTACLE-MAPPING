/**
 * Motor Control Header
 * Handles differential drive motor control for L298 driver
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Motor pin definitions (configure in main .ino)
extern int MOTOR_LEFT_IN1;
extern int MOTOR_LEFT_IN2;
extern int MOTOR_LEFT_ENABLE;
extern int MOTOR_RIGHT_IN3;
extern int MOTOR_RIGHT_IN4;
extern int MOTOR_RIGHT_ENABLE;

// Robot kinematic parameters
extern float WHEEL_BASE;
extern float WHEEL_RADIUS;
extern float SPEED_TO_PWM_RATIO;

/**
 * Initialize motor pins as outputs
 */
void initMotors() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
}

/**
 * Set motor speeds using PWM
 * @param left_pwm: PWM value for left motor (-255 to 255)
 * @param right_pwm: PWM value for right motor (-255 to 255)
 */
void setMotorSpeeds(float left_pwm, float right_pwm) {
  // Left motor direction and speed
  digitalWrite(MOTOR_LEFT_IN1, left_pwm > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_LEFT_IN2, left_pwm > 0 ? LOW : HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, abs(left_pwm));

  // Right motor direction and speed
  digitalWrite(MOTOR_RIGHT_IN4, right_pwm > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_IN3, right_pwm > 0 ? LOW : HIGH);
  analogWrite(MOTOR_RIGHT_ENABLE, abs(right_pwm));
}

/**
 * Convert differential drive velocities to wheel PWM values
 * @param v: Linear velocity (m/s)
 * @param omega: Angular velocity (rad/s)
 * @param left_pwm: Output left wheel PWM
 * @param right_pwm: Output right wheel PWM
 */
void velocityToPWM(float v, float omega, float& left_pwm, float& right_pwm) {
  // Inverse kinematics for differential drive
  float left_speed = (2.0 * v - omega * WHEEL_BASE) / 2.0;
  float right_speed = (2.0 * v + omega * WHEEL_BASE) / 2.0;

  // Convert m/s to PWM
  left_pwm = constrain(left_speed * SPEED_TO_PWM_RATIO, -255, 255);
  right_pwm = constrain(right_speed * SPEED_TO_PWM_RATIO, -255, 255);
}

/**
 * Stop all motors
 */
void stopMotors() {
  setMotorSpeeds(0, 0);
}

#endif // MOTOR_CONTROL_H
