/**
 * Kalman Filter Header
 * State estimation for robot localization (x, y, theta)
 * Fuses IMU and ArUco marker observations
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

struct RobotState {
  float x;
  float y;
  float theta;
};

struct MarkerObservation {
  float x;
  float y;
  float theta;
  int id;
};

class KalmanFilter {
private:
  // State: [x, y, theta]
  BLA::Matrix<3,1> x_state;
  BLA::Matrix<3,3> P;  // State covariance

  // Noise matrices
  BLA::Matrix<2,2> Q;  // Process noise
  BLA::Matrix<3,3> R;  // Measurement noise

  // Identity and observation matrices
  BLA::Matrix<3,3> H;
  BLA::Matrix<3,3> I;

  // Marker positions (world frame)
  int num_markers;
  float** markers;  // [num_markers][4]: {x, y, theta, id}

public:
  /**
   * Initialize Kalman filter
   * @param marker_list: Array of marker positions [x, y, theta, id]
   * @param num: Number of markers
   * @param q_v: Process noise for velocity
   * @param q_omega: Process noise for angular velocity
   * @param r_x: Measurement noise for x
   * @param r_y: Measurement noise for y
   * @param r_theta: Measurement noise for theta
   */
  KalmanFilter(float** marker_list, int num,
               float q_v, float q_omega,
               float r_x, float r_y, float r_theta) {
    // Initialize state to origin
    x_state = {0, 0, 0};

    // Initial covariance
    P = {0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001};

    // Process noise
    Q = {q_v, 0,
         0, q_omega};

    // Measurement noise
    R = {r_x, 0, 0,
         0, r_y, 0,
         0, 0, r_theta};

    // Observation model (direct measurement of state)
    H = {1, 0, 0,
         0, 1, 0,
         0, 0, 1};

    // Identity matrix
    I = {1, 0, 0,
         0, 1, 0,
         0, 0, 1};

    // Store marker positions
    markers = marker_list;
    num_markers = num;
  }

  /**
   * Prediction step using motion model
   * @param v: Linear velocity (m/s)
   * @param imu_theta: IMU heading measurement (rad)
   * @param dt: Time step (seconds)
   * @return Predicted robot state
   */
  RobotState predict(float v, float imu_theta, float dt) {
    float theta = x_state(2, 0);

    // State prediction
    x_state(0, 0) += dt * v * cos(theta);
    x_state(1, 0) += dt * v * sin(theta);
    x_state(2, 0) = imu_theta;  // Direct IMU measurement

    // Jacobian of motion model
    BLA::Matrix<3,3> F = {
      1, 0, -dt * v * sin(theta),
      0, 1,  dt * v * cos(theta),
      0, 0, 1
    };

    // Jacobian of noise
    BLA::Matrix<3,2> F_n = {
      dt * cos(theta), 0,
      dt * sin(theta), 0,
      0, dt
    };

    // Covariance prediction
    P = F * P * ~F + F_n * Q * ~F_n;

    return {x_state(0,0), x_state(1,0), x_state(2,0)};
  }

  /**
   * Update step using ArUco marker observations
   * @param observations: Array of marker observations
   * @param count: Number of observations
   * @param imu_theta: Current IMU heading (rad)
   * @return Updated robot state
   */
  RobotState update(MarkerObservation* observations, int count, float imu_theta) {
    for (int i = 0; i < count; i++) {
      // Find marker position in world frame
      float tag_x, tag_y, tag_theta;
      bool found = false;

      for (int j = 0; j < num_markers; j++) {
        if ((int)markers[j][3] == observations[i].id) {
          tag_x = markers[j][0];
          tag_y = markers[j][1];
          tag_theta = markers[j][2];
          found = true;
          break;
        }
      }

      if (!found) continue;

      // Compute robot pose from marker observation
      float dtheta = tag_theta - imu_theta;
      float cos_world = cos(tag_theta);
      float sin_world = sin(tag_theta);
      float cos_robot = cos(dtheta);
      float sin_robot = sin(dtheta);

      // Transformation: World ← Tag ← Robot
      BLA::Matrix<3,3> R_wt = {
        cos_world, -sin_world, tag_x,
        sin_world,  cos_world, tag_y,
        0, 0, 1
      };

      BLA::Matrix<3,3> R_ct = {
        cos_robot, -sin_robot, observations[i].x,
        sin_robot,  cos_robot, observations[i].y,
        0, 0, 1
      };

      BLA::Matrix<3,3> z_matrix = R_wt * Inverse(R_ct);

      // Extract measurement
      BLA::Matrix<3,1> z = {
        z_matrix(0,2),
        z_matrix(1,2),
        atan2(z_matrix(1,0), z_matrix(0,0))
      };

      // Kalman update
      BLA::Matrix<3,3> S = H * P * ~H + R;
      BLA::Matrix<3,3> K = P * ~H * Inverse(S);

      x_state = x_state + K * (z - H * x_state);
      P = (I - K * H) * P;
    }

    return {x_state(0,0), x_state(1,0), x_state(2,0)};
  }

  /**
   * Get current state estimate
   */
  RobotState getState() {
    return {x_state(0,0), x_state(1,0), x_state(2,0)};
  }
};

#endif // KALMAN_FILTER_H
