# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an autonomous mobile robot navigation system that integrates visual localization using ArUco markers, inertial sensing (IMU), and LIDAR-based obstacle avoidance for indoor environments. The system uses a **hybrid computational architecture**:

- **Raspberry Pi 4**: Handles high-level perception (ArUco detection, pose estimation, LIDAR processing)
- **Arduino Uno R3**: Manages low-level control (Kalman filter, sensor fusion, motor control)

The robot is a differential-drive platform equipped with:
- Arducam 16MP camera for visual input
- MPU6050 IMU for orientation sensing
- RPLIDAR A1M8 for obstacle detection
- L298 motor driver controlling two DC motors

## System Architecture

### Communication Flow

```
Raspberry Pi → Arduino: Serial @ 9600 baud (Task 1) or 115200 baud (Task 2)
- ArUco pose data: "{count}x{x}y{y}t{theta}i{marker_id}x...#"
- LIDAR obstacle data: "s{angle}s{angle}s...#" (free sectors)

Arduino → Raspberry Pi: Status/debug messages via Serial.println()
```

### Hardware Coordinate Frames

**Camera-to-Body Transformation (Tbc):**
```python
Tbc = [[0, 0, 1, 0.10],
       [-1, 0, 0, -0.02],
       [0, -1, 0, 0.022],
       [0, 0, 0, 1]]
```
This fixed transformation converts camera frame measurements to robot body frame.

**Robot Kinematics:**
- Wheel radius: 0.032 m
- Wheelbase: 0.13 m
- Max speed: 0.836 m/s
- Max angular velocity: 12.863 rad/s

## Code Structure

### Task 1: ArUco-Based Localization (first_task/)

**Python (Raspberry Pi):** `aruco_with_homo.py`
- Detects 4×4 ArUco markers using OpenCV
- Estimates 6-DOF pose via `cv.solvePnPRansac()`
- Validates detections using reprojection error thresholds
- Sends pose to Arduino via serial

**Arduino:** `aruco_with_homo.ino`
- Implements Kalman filter for state estimation (x, y, θ)
- Fuses ArUco observations with IMU yaw measurements
- Uses BasicLinearAlgebra library for matrix operations
- Proportional controller navigates through waypoints
- Tracks velocity via IMU accelerometer integration

**Key Implementation Details:**
- ArUco marker size: 0.096 m (96 mm)
- Marker validation: Rejects if area/error thresholds exceeded
- Process noise (Q) deliberately increased to trust visual updates over odometry
- Measurement noise (R) tuned for visual observations

### Task 2: SLAM with LIDAR Obstacle Avoidance (second_task/)

**Python (Raspberry Pi):** `SLAM.py`
- Runs two threads: LIDAR scanning + ArUco detection
- Segments 360° LIDAR scan into 5° sectors
- Inflates obstacles by ±20° (25° left + 25° right) for safety buffer
- Identifies free sectors within 1.5m range
- Rate-limits ArUco processing (every 3rd cycle)

**Arduino:** `SLAM.ino`
- Same Kalman filter as Task 1
- Dynamic Window Approach-inspired navigation:
  - Receives free sectors from LIDAR
  - Searches for closest free direction toward goal within 0.8m radius
  - Updates local goal every time robot reaches waypoint
  - Uses 3-second transition delay between goals
- Averages commanded velocity with estimated velocity for smoother control

**Key Differences from Task 1:**
- Higher baud rate (115200 vs 9600)
- Dynamic obstacle avoidance vs direct waypoint navigation
- Processes LIDAR data for real-time path planning

### Calibration (calibrate/)

**calibrate.py**
- Camera calibration using 4×4 chessboard pattern
- Computes camera intrinsic matrix (mtx) and distortion coefficients (dist)
- These calibration parameters are hardcoded in both Python scripts

**Calibration Results Used:**
```python
mtx = [[583.39999978, 0, 317.26971098],
       [0, 568.87700109, 226.74554456],
       [0, 0, 1]]
dist = [[4.37606760e-01, -2.53165723e+00, -1.30981396e-03,
         1.46723672e-02, 4.64273442e+00]]
```

### Visualization (animate.py)

Real-time polar plot visualization of LIDAR scans using matplotlib animation.

## ArUco Marker Setup

Markers must be pre-placed at known world coordinates. The marker database is hardcoded in Arduino `.ino` files:

```cpp
// Format: {x_world, y_world, theta_world, marker_id}
float markers[num_marker][4] = {
  {1*0.6, 0*0.6, -M_PI/2, 7},
  {3*0.6, -2*0.6, -M_PI/2, 9},
  // ... add more as needed
};
```

**Important:** Update this array to match your physical marker placement before deployment.

## Kalman Filter Implementation

The filter runs on Arduino and estimates robot state [x, y, θ].

**Prediction Step:**
```cpp
x_k = x_{k-1} + v*dt*cos(θ)
y_k = y_{k-1} + v*dt*sin(θ)
θ_k = θ_IMU  // Direct IMU measurement
```

**Update Step (when ArUco detected):**
- Computes robot pose from marker observation
- Transforms: World ← Tag ← Camera ← Robot
- Applies Kalman update to correct prediction

**Tuning Parameters:**
- `Q_t`: Process noise (higher = trust predictions less)
- `R_t`: Measurement noise (lower = trust ArUco more)
- Located at top of Arduino `.ino` files

## Serial Protocol

**ArUco Data Format:**
```
{count}x{x_val}y{y_val}t{theta_val}i{id}x{x_val}y{y_val}t{theta_val}i{id}#
```
Example: `2x0.52y1.34t0.98i7x0.61y1.28t1.02i9#`
- Count: Number of markers detected
- Multiple markers concatenated (x/y/t/i repeats)
- Terminated with '#'

**LIDAR Free Sectors Format:**
```
s{angle}s{angle}s{angle}s...#
```
Example: `s0s5s10s15s340s345s350s355#`
- Each value is a free 5° sector angle
- Only sectors without obstacles within 1m are included

## Running the System

### Prerequisites

**Raspberry Pi:**
```bash
pip install opencv-python numpy picamera2 pyserial rplidar-roboticia
```

**Arduino:**
- Install BasicLinearAlgebra library
- Install MPU6050_light library

### Execution Steps

1. **Upload Arduino Code:**
   - Open `first_task/aruco_with_homo/aruco_with_homo.ino` OR `second_task/SLAM/SLAM.ino`
   - Update marker positions in the `markers[]` array
   - Update goal waypoints in `goal[]` array (Task 1) or `goalx/goaly` (Task 2)
   - Upload to Arduino Uno R3

2. **Run Python Script on Raspberry Pi:**
   ```bash
   # For Task 1 (ArUco only)
   python3 final_codes/first_task/aruco_with_homo.py

   # For Task 2 (ArUco + LIDAR)
   python3 final_codes/second_task/SLAM.py
   ```

3. **Startup Handshake:**
   - Python sends "START\n" after 2-second delay
   - Arduino blocks in setup() until receiving "START"

4. **Monitor via Serial:**
   - Arduino prints state estimates: `x y theta`
   - Watch for "transition" messages when waypoints are reached
   - Final message: "Goal reached!" when all waypoints completed

### Important Notes

- **Serial Port:** Default is `/dev/ttyACM0` for Arduino and `/dev/ttyUSB0` for LIDAR. Update if different.
- **Camera Calibration:** If using different camera, re-run `calibrate/calibrate.py` and update `mtx` and `dist` arrays.
- **Marker Size:** Code assumes 96mm ArUco markers. Update `objectPoints` in Python if different.
- **Baud Rate Mismatch:** Task 1 uses 9600 baud, Task 2 uses 115200. Ensure Python and Arduino match.

## Control Parameters

**Proportional Controller Gains (Arduino):**
```cpp
const float kp = 0.9;   // Distance gain
const float ka = 8-20;  // Angle gain (varies by task)
float threshold = 0.22-0.25;  // Goal reached threshold (meters)
```

**Motor Control:**
- PWM mapping: `speed_m/s * 150/0.492` → PWM value (0-255)
- Differential drive: `ω_L = (2v - ωb)/2`, `ω_R = (2v + ωb)/2`

## Common Modifications

### Adding New Waypoints (Task 1)

Edit Arduino file:
```cpp
float goal[NUM_WAYPOINTS][2] = {
  {0.353, -0.353},
  {0.806, -0.564},
  {1.6, 0},
  // Add more {x, y} pairs
};
```

### Changing Final Goal (Task 2)

Edit Arduino file:
```cpp
float goalx = 3*0.6;  // Final x coordinate
float goaly = -1*0.6; // Final y coordinate
```

### Tuning Obstacle Avoidance

Edit Python `SLAM.py`:
```python
MAX_DIST = 1500  # mm, obstacle detection range
SECTOR_WIDTH = 5  # degrees per sector
```

Edit Arduino `SLAM.ino`:
```cpp
// In dynamic_window() function:
l_x = ss.x + 0.8*cos(...)  // Local goal search radius (0.8m)
```

### Adjusting Kalman Filter Confidence

Higher process noise → trust visual updates more:
```cpp
BLA::Matrix<2,2> Q_t = {0.04, 0, 0, 0.000064};  // Increase diagonal values
```

Lower measurement noise → trust ArUco more:
```cpp
BLA::Matrix<3,3> R_t = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.1};  // Decrease values
```

## Debugging Tips

- **No ArUco Detection:** Check lighting, marker size assumptions, camera calibration
- **Serial Communication Issues:** Verify baud rates match, check USB permissions on Pi
- **Kalman Filter Divergence:** Review Q/R tuning, check IMU calibration
- **Poor Navigation:** Tune kp/ka gains, verify goal coordinates in robot frame
- **LIDAR Not Working:** Check port `/dev/ttyUSB0`, ensure motor spinning, verify baudrate 115200
- **Robot Drifting:** Re-calibrate IMU (`mpu.calcOffsets()`), check motor PWM calibration

## Key Dependencies & Libraries

- **OpenCV (cv2):** ArUco detection, pose estimation, calibration
- **picamera2:** Raspberry Pi camera interface
- **rplidar:** RPLIDAR A1M8 driver
- **BasicLinearAlgebra:** Arduino matrix operations for Kalman filter
- **MPU6050_light:** IMU interface library
- **Wire.h:** I2C communication with IMU
