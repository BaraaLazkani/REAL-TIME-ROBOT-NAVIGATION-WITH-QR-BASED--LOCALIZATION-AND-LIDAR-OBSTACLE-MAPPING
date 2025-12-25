/**
 * Task 2: SLAM with LIDAR Navigation
 *
 * Implements goal-directed navigation with LIDAR-based obstacle avoidance.
 * Uses Dynamic Window Approach-inspired algorithm for local path planning.
 *
 * Hardware:
 * - Arduino Uno R3
 * - MPU6050 IMU
 * - L298 Motor Driver
 * - DC Motors
 * - RPLIDAR A1M8 (processed by Raspberry Pi)
 *
 * Communication: Serial @ 115200 baud with Raspberry Pi
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include "../common/kalman_filter.h"
#include "../common/motor_control.h"
#include "../common/navigation.h"

// ============ CONFIGURATION ============

// Motor pins
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
float NAV_KA = 8.0;  // Lower gain for smoother turning
float NAV_THRESHOLD = 0.22;
float MAX_LINEAR_VEL = 0.836;
float MAX_ANGULAR_VEL = 12.863;

// Kalman filter noise parameters
float Q_V = 0.04;
float Q_OMEGA = 0.000064;
float R_X = 0.0001;
float R_Y = 0.0001;
float R_THETA = 0.1;

// ArUco marker positions
const int NUM_MARKERS = 3;
float marker_data[NUM_MARKERS][4] = {
  {0.6, 0.0, -1.5708, 7},
  {1.8, -1.2, -1.5708, 9},
  {1.2, -0.6, -1.5708, 8}
};

// Final goal
float GOAL_X = 1.8;
float GOAL_Y = -0.6;

// LIDAR parameters
const int MAX_FREE_SECTORS = 80;
int free_sectors[MAX_FREE_SECTORS];
int num_free_sectors = 0;
bool lidar_ready = false;
bool local_goal_set = false;

// Local goal (intermediate)
float local_goal_x = 10.0;
float local_goal_y = 10.0;
float LOCAL_GOAL_RADIUS = 0.8;  // Search radius

// Timing
unsigned long transition_timer = 0;
unsigned long TRANSITION_DELAY = 3000;  // ms

// ============ GLOBAL VARIABLES ============
MPU6050 mpu(Wire);
KalmanFilter* kf;

RobotState current_state = {0, 0, 0};
float real_velocity = 0;
unsigned long last_time = 0;

String serial_buffer = "";
MarkerObservation observations[10];
int obs_count = 0;

// ============ SETUP ============
void setup() {
  Serial.begin(115200);

  // Wait for START signal
  while(!Serial.available() || Serial.readStringUntil('\n') != "START");

  // Initialize motors
  initMotors();

  // Initialize IMU
  Wire.begin();
  byte imu_status = mpu.begin();
  while(imu_status != 0) {}
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
  Serial.println("Task 2 Arduino Ready");
}

// ============ MAIN LOOP ============
void loop() {
  // Parse serial data
  parseSerialData();

  // Update local goal if LIDAR data available
  if (lidar_ready && !local_goal_set) {
    updateLocalGoal();
    local_goal_set = true;
  }

  // Time delta
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();

  // Update velocity from IMU
  real_velocity += -mpu.getAccX() * dt * 9.81;
  if (real_velocity < 0) real_velocity = 0;

  // Compute navigation command
  VelocityCommand cmd = computeVelocity(
    current_state.x, current_state.y, current_state.theta,
    local_goal_x, local_goal_y
  );

  // Stop if no LIDAR data
  if (!lidar_ready) {
    cmd.v = 0;
    cmd.omega = 0;
  }

  // Kalman filter prediction
  mpu.update();
  current_state = kf->predict((real_velocity + cmd.v) * 0.5,
                              mpu.getAngleZ() * PI / 180.0, dt);

  // Kalman filter update
  if (obs_count > 0) {
    current_state = kf->update(observations, obs_count,
                               mpu.getAngleZ() * PI / 180.0);
    obs_count = 0;
  }

  // Print state
  Serial.print(current_state.x);
  Serial.print(" ");
  Serial.print(current_state.y);
  Serial.print(" ");
  Serial.println(current_state.theta);

  // Execute motion
  if (!cmd.done || (millis() - transition_timer) < TRANSITION_DELAY) {
    float left_pwm, right_pwm;
    velocityToPWM(cmd.v, cmd.omega, left_pwm, right_pwm);
    setMotorSpeeds(left_pwm, right_pwm);
  } else {
    // Local goal reached
    transition_timer = millis();
    local_goal_set = false;
    Serial.println("Local goal reached");

    // Check if final goal reached
    float dist_to_goal = sqrt(
      (current_state.x - GOAL_X) * (current_state.x - GOAL_X) +
      (current_state.y - GOAL_Y) * (current_state.y - GOAL_Y)
    );

    if (dist_to_goal < NAV_THRESHOLD) {
      stopMotors();
      Serial.println("Final goal reached!");
      while(1) {}
    }
  }

  delay(10);
}

// ============ DYNAMIC WINDOW ============
void updateLocalGoal() {
  // Check if close to final goal
  float dist = sqrt(
    (current_state.x - GOAL_X) * (current_state.x - GOAL_X) +
    (current_state.y - GOAL_Y) * (current_state.y - GOAL_Y)
  );

  if (dist < 1.0) {
    local_goal_x = GOAL_X;
    local_goal_y = GOAL_Y;
    return;
  }

  // Search free sectors for best direction
  float min_dist = 10000;

  for (int i = 0; i < num_free_sectors; i++) {
    float sector_angle = free_sectors[i] * PI / 180.0;
    float robot_angle = 3 * PI / 2 - sector_angle + current_state.theta;

    float lx = current_state.x + LOCAL_GOAL_RADIUS * cos(robot_angle);
    float ly = current_state.y + LOCAL_GOAL_RADIUS * sin(robot_angle);

    float d = sqrt((lx - GOAL_X) * (lx - GOAL_X) +
                   (ly - GOAL_Y) * (ly - GOAL_Y));

    if (d < min_dist) {
      min_dist = d;
      local_goal_x = lx;
      local_goal_y = ly;
    }
  }

  Serial.print("Local goal: ");
  Serial.print(local_goal_x);
  Serial.print(", ");
  Serial.println(local_goal_y);
}

// ============ SERIAL PARSING ============
void parseSerialData() {
  if (!Serial.available()) return;

  serial_buffer = Serial.readStringUntil('#');
  if (serial_buffer.length() == 0) return;

  Serial.println("RX: " + serial_buffer);

  // LIDAR data (starts with 's')
  if (serial_buffer.charAt(0) == 's') {
    parseLidarData();
  }
  // ArUco data (starts with digit)
  else if (isDigit(serial_buffer.charAt(0))) {
    parseArucoData();
  }
}

void parseLidarData() {
  num_free_sectors = 0;
  lidar_ready = true;

  while (serial_buffer.length() > 1 && num_free_sectors < MAX_FREE_SECTORS) {
    serial_buffer.remove(0, 1);  // Remove 's'

    String sector_str = "";
    while (serial_buffer.length() > 0 && serial_buffer.charAt(0) != 's') {
      sector_str += serial_buffer.charAt(0);
      serial_buffer.remove(0, 1);
    }

    if (sector_str.length() > 0) {
      free_sectors[num_free_sectors] = sector_str.toInt();
      num_free_sectors++;
    }
  }
}

void parseArucoData() {
  int count = serial_buffer.charAt(0) - '0';
  serial_buffer.remove(0, 1);
  obs_count = 0;

  for (int i = 0; i < count && obs_count < 10; i++) {
    if (serial_buffer.charAt(0) != 'x') break;

    float x = parseValue('x', 'y');
    float y = parseValue('y', 't');
    float theta = parseValue('t', 'i');
    int id = parseValue('i', 'x').toInt();

    observations[obs_count] = {x, y, theta, id};
    obs_count++;
  }
}

String parseValue(char start, char end) {
  serial_buffer.remove(0, 1);
  String value = "";
  while (serial_buffer.length() > 0 && serial_buffer.charAt(0) != end) {
    value += serial_buffer.charAt(0);
    serial_buffer.remove(0, 1);
  }
  return value;
}
