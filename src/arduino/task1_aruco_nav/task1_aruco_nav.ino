/**
 * Task 1: ArUco Marker-Based Navigation
 *
 * Implements waypoint navigation using ArUco marker localization
 * fused with IMU data via Kalman filter.
 *
 * Hardware:
 * - Arduino Uno R3
 * - MPU6050 IMU
 * - L298 Motor Driver
 * - DC Motors
 *
 * Communication: Serial @ 9600 baud with Raspberry Pi
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include "../common/kalman_filter.h"
#include "../common/motor_control.h"
#include "../common/navigation.h"

// ============ CONFIGURATION ============
// Update these values to match config/task1_config.yaml

// Motor pins (from config/robot_params.yaml)
int MOTOR_LEFT_IN1 = 7;
int MOTOR_LEFT_IN2 = 6;
int MOTOR_LEFT_ENABLE = 5;
int MOTOR_RIGHT_IN3 = 8;
int MOTOR_RIGHT_IN4 = 9;
int MOTOR_RIGHT_ENABLE = 10;

// Robot parameters
float WHEEL_BASE = 0.13;
float WHEEL_RADIUS = 0.032;
float SPEED_TO_PWM_RATIO = 304.878;

// Navigation parameters
float NAV_KP = 0.9;
float NAV_KA = 20.0;
float NAV_THRESHOLD = 0.25;
float MAX_LINEAR_VEL = 0.836;
float MAX_ANGULAR_VEL = 12.863;

// Kalman filter noise parameters
float Q_V = 0.04;
float Q_OMEGA = 0.000064;
float R_X = 0.0001;
float R_Y = 0.0001;
float R_THETA = 0.1;

// ArUco marker positions [x, y, theta, id]
const int NUM_MARKERS = 3;
float marker_data[NUM_MARKERS][4] = {
  {1.2, 1.6, -1.5708, 6},
  {1.6, -0.8, -1.5708, 7},
  {0.8, 0.0, -1.5708, 8}
};

// Waypoints [x, y]
const int NUM_WAYPOINTS = 3;
float waypoints[NUM_WAYPOINTS][2] = {
  {0.353, -0.353},
  {0.806, -0.564},
  {1.6, 0.0}
};

// ============ GLOBAL VARIABLES ============
MPU6050 mpu(Wire);
KalmanFilter* kf;

// State variables
RobotState current_state = {0, 0, 0};
int current_waypoint = 0;
float real_velocity = 0;
unsigned long last_time = 0;

// Serial parsing
String serial_buffer = "";
MarkerObservation observations[10];
int obs_count = 0;

// ============ SETUP ============
void setup() {
  Serial.begin(9600);

  // Wait for START signal from Raspberry Pi
  while(!Serial.available() || Serial.readStringUntil('\n') != "START");

  // Initialize motors
  initMotors();

  // Initialize IMU
  Wire.begin();
  byte imu_status = mpu.begin();
  while(imu_status != 0) {}  // Wait for IMU
  delay(1000);
  mpu.calcOffsets();

  // Initialize Kalman filter
  float* marker_ptrs[NUM_MARKERS];
  for(int i = 0; i < NUM_MARKERS; i++) {
    marker_ptrs[i] = marker_data[i];
  }
  kf = new KalmanFilter(marker_ptrs, NUM_MARKERS,
                        Q_V, Q_OMEGA, R_X, R_Y, R_THETA);

  last_time = millis();
  Serial.println("Task 1 Arduino Ready");
}

// ============ MAIN LOOP ============
void loop() {
  mpu.update();

  // Read serial data from Raspberry Pi
  parseSerialData();

  // Time delta
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();

  // Update velocity estimate from IMU
  real_velocity += -mpu.getAccX() * dt * 9.81;
  if (real_velocity < 0) real_velocity = 0;

  // Get current goal
  float goal_x = waypoints[current_waypoint][0];
  float goal_y = waypoints[current_waypoint][1];

  // Compute navigation command
  VelocityCommand cmd = computeVelocity(
    current_state.x, current_state.y, current_state.theta,
    goal_x, goal_y
  );

  // Kalman filter prediction
  current_state = kf->predict(real_velocity, mpu.getAngleZ() * PI / 180.0, dt);

  // Kalman filter update (if markers observed)
  if (obs_count > 0) {
    current_state = kf->update(observations, obs_count, mpu.getAngleZ() * PI / 180.0);
    obs_count = 0;
  }

  // Print state
  Serial.print(current_state.x);
  Serial.print(" ");
  Serial.print(current_state.y);
  Serial.print(" ");
  Serial.println(current_state.theta);

  // Execute motion or check goal reached
  if (!cmd.done) {
    float left_pwm, right_pwm;
    velocityToPWM(cmd.v, cmd.omega, left_pwm, right_pwm);
    setMotorSpeeds(left_pwm, right_pwm);
  } else {
    // Goal reached, move to next waypoint
    current_waypoint++;
    Serial.print("Waypoint ");
    Serial.print(current_waypoint);
    Serial.println(" reached");

    if (current_waypoint >= NUM_WAYPOINTS) {
      stopMotors();
      Serial.println("All waypoints complete!");
      while(1) {}  // Stop execution
    }
  }

  delay(10);
}

// ============ SERIAL PARSING ============
void parseSerialData() {
  if (!Serial.available()) return;

  serial_buffer = Serial.readStringUntil('#');
  if (serial_buffer.length() == 0) return;

  Serial.println("Received: " + serial_buffer);

  // Parse marker count
  int count = serial_buffer.charAt(0) - '0';
  serial_buffer.remove(0, 1);

  obs_count = 0;

  // Parse each marker observation
  for (int i = 0; i < count && obs_count < 10; i++) {
    if (serial_buffer.length() == 0) break;
    if (serial_buffer.charAt(0) != 'x') break;

    // Parse x, y, theta, id
    float x = parseValue('x', 'y');
    float y = parseValue('y', 't');
    float theta = parseValue('t', 'i');
    int id = parseValue('i', 'x').toInt();

    observations[obs_count] = {x, y, theta, id};
    obs_count++;
  }
}

String parseValue(char start, char end) {
  serial_buffer.remove(0, 1);  // Remove start char
  String value = "";
  while (serial_buffer.length() > 0 && serial_buffer.charAt(0) != end) {
    value += serial_buffer.charAt(0);
    serial_buffer.remove(0, 1);
  }
  return value;
}
