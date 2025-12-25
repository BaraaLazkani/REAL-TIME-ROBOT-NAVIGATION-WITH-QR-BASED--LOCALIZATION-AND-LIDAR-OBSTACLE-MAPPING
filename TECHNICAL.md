# Technical Documentation

## Table of Contents
- [System Architecture](#system-architecture)
- [Theoretical Background](#theoretical-background)
- [Implementation Details](#implementation-details)
- [Algorithms](#algorithms)
- [Experimental Results](#experimental-results)
- [Performance Analysis](#performance-analysis)

---

## System Architecture

### Overview

The system employs a **hybrid computational architecture** that distributes processing across two platforms:

| Platform | Role | Responsibilities |
|----------|------|------------------|
| **Raspberry Pi 4** | High-level perception | ArUco detection, pose estimation, LIDAR processing |
| **Arduino Uno R3** | Low-level control | Kalman filtering, sensor fusion, motor control |

**Rationale**: This division leverages the Raspberry Pi's computational power for vision tasks while ensuring real-time control loop execution on the Arduino's deterministic microcontroller.

### Hardware Components

```
┌─────────────────────────────────────────────────────────┐
│                    Robot Platform                        │
│                                                          │
│  ┌─────────────┐         ┌──────────────┐              │
│  │ Raspberry   │  USB    │   Arduino    │              │
│  │   Pi 4      │◄───────►│   Uno R3     │              │
│  │             │ Serial  │              │              │
│  └──────┬──────┘         └───────┬──────┘              │
│         │                        │                      │
│    CSI  │                   I2C  │  PWM                │
│         ▼                        ▼   ▼                 │
│  ┌─────────┐            ┌────────┐ ┌────────┐         │
│  │ Arducam │            │MPU6050 │ │ L298   │         │
│  │ 16MP    │            │  IMU   │ │ Motor  │         │
│  └─────────┘            └────────┘ │ Driver │         │
│                                    └───┬────┘         │
│  ┌─────────┐                          │               │
│  │ RPLIDAR │                          ▼               │
│  │  A1M8   │                    ┌──────────┐          │
│  └─────────┘                    │ DC Motors│          │
│       USB                       │ (2x 1:48)│          │
│                                 └──────────┘          │
└─────────────────────────────────────────────────────────┘
```

### Software Architecture

#### Raspberry Pi Stack
```
┌────────────────────────────────────────┐
│   Task 1/2 Main Scripts                │
│   (task1_aruco_localization.py)        │
├────────────────────────────────────────┤
│   Common Modules:                      │
│   • camera.py - Camera interface       │
│   • aruco_detector.py - Marker detect  │
│   • serial_comm.py - Arduino comms     │
│   • lidar_processor.py - LIDAR proc    │
│   • config_loader.py - YAML loading    │
├────────────────────────────────────────┤
│   External Libraries:                  │
│   OpenCV • NumPy • Picamera2           │
│   PySerial • RPLidar • PyYAML          │
└────────────────────────────────────────┘
```

#### Arduino Stack
```
┌────────────────────────────────────────┐
│   Task 1/2 Main Sketch                 │
│   (task1_aruco_nav.ino)                │
├────────────────────────────────────────┤
│   Common Headers:                      │
│   • kalman_filter.h - State estimate   │
│   • motor_control.h - Diff drive       │
│   • navigation.h - Path planning       │
├────────────────────────────────────────┤
│   External Libraries:                  │
│   BasicLinearAlgebra • MPU6050_light   │
│   Wire (I2C)                           │
└────────────────────────────────────────┘
```

---

## Theoretical Background

### 1. Differential Drive Kinematics

The robot uses a **differential-drive** configuration with two independently controlled wheels.

**Forward Kinematics:**
```
v = (v_R + v_L) / 2
ω = (v_R - v_L) / b

where:
  v_R, v_L = right/left wheel velocities (m/s)
  v = linear velocity (m/s)
  ω = angular velocity (rad/s)
  b = wheelbase (0.13 m)
```

**Inverse Kinematics:**
```
v_R = v + (b/2)ω
v_L = v - (b/2)ω
```

**Pose Update:**
```
ẋ = v cos(θ)
ẏ = v sin(θ)
θ̇ = ω
```

### 2. ArUco Marker Detection & Pose Estimation

#### Detection Pipeline

1. **Image Preprocessing**: Convert to grayscale
2. **Contour Extraction**: Find quadrilaterals using adaptive thresholding
3. **Perspective Normalization**: Warp to canonical view
4. **Bit Decoding**: Extract binary pattern from grid
5. **ID Matching**: Compare against dictionary with Hamming distance check

#### Pose Estimation (PnP Problem)

Given:
- 4 marker corners in image: `p_i = (u_i, v_i)`
- Known 3D coordinates: `P_i = (0,0,0), (s,0,0), (s,s,0), (0,s,0)`
- Camera matrix `K` and distortion coefficients `d`

Solve for rotation `R` and translation `t`:

```
s · [u; v; 1] = K · [R | t] · [X; Y; Z; 1]
```

**Method**: `solvePnPRansac` with IPPE_SQUARE flag for robust estimation

**Validation**: Reject if reprojection error exceeds threshold:
```
e_mean = (1/4) Σ ||p_i - p̂_i||

Threshold:
  < 1.4 pixels  (for small/distant markers)
  < 4.5 pixels  (for large/close markers)
```

#### Coordinate Frame Transformation

**Camera → Body Frame:**

The camera is mounted with offset and rotation. Transformation matrix `T_bc`:

```
T_bc = [ 0   0   1  0.10]
       [-1   0   0 -0.02]
       [ 0  -1   0  0.022]
       [ 0   0   0   1  ]
```

**Marker → World Frame:**

Given marker pose `T_cm` (camera to marker) and world position `T_wm`:

```
T_wb = T_wm · T_mc · T_cb

Robot pose:
  x = T_wb[0,3]
  y = T_wb[1,3]
  θ = atan2(T_wb[1,0], T_wb[0,0])
```

### 3. Kalman Filter for Sensor Fusion

#### State Vector
```
x = [x, y, θ]ᵀ

where:
  x, y = position in world frame (m)
  θ = heading angle (rad)
```

#### Motion Model (Prediction Step)

```
x_k = x_{k-1} + v·Δt·cos(θ)
y_k = y_{k-1} + v·Δt·sin(θ)
θ_k = θ_IMU  (direct measurement)
```

**Jacobian F:**
```
F = [1   0  -v·Δt·sin(θ)]
    [0   1   v·Δt·cos(θ)]
    [0   0         1     ]
```

**Process Noise Q:**
```
Q = [σ_v²     0   ]
    [  0   σ_ω²  ]

where:
  σ_v = 0.04 (velocity noise)
  σ_ω = 0.000064 (angular velocity noise)
```

**Covariance Prediction:**
```
P_{k|k-1} = F·P_{k-1}·Fᵀ + F_n·Q·F_nᵀ
```

#### Observation Model (Update Step)

**Measurement:** Robot pose from ArUco marker observation

```
z = [x_obs, y_obs, θ_obs]ᵀ
```

**Observation matrix H:** Identity (direct measurement of state)

**Measurement Noise R:**
```
R = [0.0001    0        0   ]
    [   0   0.0001      0   ]
    [   0      0      0.1   ]
```

**Kalman Gain:**
```
K = P_{k|k-1}·Hᵀ·(H·P_{k|k-1}·Hᵀ + R)⁻¹
```

**State Update:**
```
x_{k|k} = x_{k|k-1} + K·(z - H·x_{k|k-1})
P_{k|k} = (I - K·H)·P_{k|k}
```

**Design Choice**: High process noise forces filter to trust visual observations over odometry, compensating for wheel slip and IMU drift.

### 4. Proportional Controller for Navigation

**Control Law:**
```
Distance to goal: ρ = √[(x_g - x)² + (y_g - y)²]
Angle to goal:    α = -θ + atan2(y_g - y, x_g - x)

Commands:
  v = k_p · ρ
  ω = k_a · α

Constraints:
  |v| ≤ 0.836 m/s
  |ω| ≤ 12.863 rad/s
```

**Gains:**
- Task 1: `k_p = 0.9`, `k_a = 20.0` (aggressive turning)
- Task 2: `k_p = 0.9`, `k_a = 8.0` (smoother for obstacle avoidance)

**Goal Reached:** `ρ < threshold` (0.22-0.25 m)

### 5. LIDAR-Based Obstacle Avoidance

Inspired by **Dynamic Window Approach (DWA)**.

#### Algorithm Steps

**1. Sector Segmentation:**
```
360° scan → 72 sectors (5° each)
```

**2. Obstacle Detection:**
```
For each LIDAR point (angle, distance):
  sector = ⌊angle / 5°⌋ × 5°
  if min_dist ≤ distance ≤ max_dist:
    mark sector as OCCUPIED
```

**3. Safety Inflation:**
```
For each occupied sector:
  mark sectors [sector - 20°, sector + 20°] as BLOCKED
```

Creates **±20° buffer** around obstacles.

**4. Free Sector Identification:**
```
FREE_SECTORS = ALL_SECTORS \ BLOCKED_SECTORS
```

**5. Local Goal Selection:**
```
For each free sector s:
  θ_robot = 3π/2 - s + θ_current
  candidate = (x + r·cos(θ_robot), y + r·sin(θ_robot))

Select candidate closest to global goal within radius r = 0.8m
```

**6. Iterative Execution:**
- Navigate to local goal
- Every 0.5m: recompute free sectors and update local goal
- Repeat until global goal reached

---

## Implementation Details

### Serial Communication Protocol

**Raspberry Pi → Arduino**

**ArUco Data Format:**
```
{count}x{x}y{y}t{theta}i{id}x{x}y{y}t{theta}i{id}#

Example: "2x0.52y1.34t0.98i7x0.61y1.28t1.02i9#"
```

**LIDAR Data Format:**
```
s{angle}s{angle}s{angle}#

Example: "s0s5s10s15s340s345s350s355#"
```

**Baud Rates:**
- Task 1: 9600 (sufficient for ArUco only)
- Task 2: 115200 (needed for LIDAR + ArUco)

### Configuration Management

All parameters externalized to YAML files:

**Example: `task1_config.yaml`**
```yaml
markers:
  - id: 6
    x: 1.2
    y: 1.6
    theta: -1.5708

waypoints:
  - {x: 0.353, y: -0.353}
  - {x: 0.806, y: -0.564}

navigation:
  kp: 0.9
  ka: 20.0
  threshold: 0.25

kalman_filter:
  process_noise_q:
    v: 0.04
    omega: 0.000064
```

**Benefits:**
- Change parameters without recompiling
- Easy experimentation with different gains
- Version control friendly

---

## Algorithms

### Task 1: ArUco-Based Waypoint Navigation

```
INITIALIZE:
  Load configurations
  Initialize camera, IMU, serial
  Create Kalman filter
  waypoint_index = 0

LOOP:
  Capture frame
  Detect ArUco markers

  FOR each detected marker:
    Estimate 6-DOF pose (R, t)
    Validate using reprojection error
    Transform to robot frame
    Send to Arduino via serial

  IF markers detected:
    Arduino updates Kalman filter
    Get filtered pose (x, y, θ)

  Compute velocity command (v, ω) toward current waypoint

  Convert to wheel speeds:
    v_L = v - (b/2)ω
    v_R = v + (b/2)ω

  IF distance to waypoint < threshold:
    waypoint_index++
    IF all waypoints visited:
      STOP
```

### Task 2: LIDAR Navigation with Obstacle Avoidance

```
INITIALIZE:
  Load configurations
  Initialize camera, IMU, LIDAR, serial
  Create Kalman filter
  local_goal = None

LOOP:
  // LIDAR Processing (every cycle)
  Get latest LIDAR scan
  Compute free sectors
  Send to Arduino

  IF free sectors available AND local_goal invalid:
    Find free sector closest to global goal
    Set local_goal = position in that direction (0.8m away)

  // ArUco Processing (every 3rd cycle)
  IF cycle % 3 == 0:
    Capture frame
    Detect ArUco markers
    FOR first detected marker:
      Estimate pose
      Transform to robot frame
      Send to Arduino

  // Navigation
  Compute velocity (v, ω) toward local_goal

  IF LIDAR not ready:
    v = 0, ω = 0  // Safety stop

  Convert to wheel speeds and actuate motors

  IF distance to local_goal < threshold:
    local_goal = None  // Force recomputation

    IF distance to global_goal < threshold:
      STOP - Goal reached!
```

---

## Experimental Results

### Test Environment
- **Indoor lab** with controlled lighting
- **Floor area**: ~6m × 4m
- **ArUco markers**: 96mm, printed on white paper
- **Marker placement**: Taped to walls at known positions

### Task 1: Waypoint Navigation Results

**Test Configuration:**
- 3 waypoints: (0.35, -0.35), (0.81, -0.56), (1.6, 0.0)
- 3 ArUco markers: IDs 6, 7, 8
- Navigation gains: k_p=0.9, k_a=20.0

**Performance Metrics:**

| Metric | Value |
|--------|-------|
| Average position error | < 10 cm |
| Average heading error | < 5° |
| Waypoint reach accuracy | 95% (< 25cm) |
| Marker detection rate | 15-20 Hz |
| Kalman filter update rate | 10 Hz |
| Control loop frequency | 100 Hz (Arduino) |

**Observations:**
- ✅ Smooth trajectory following
- ✅ Effective drift correction via visual updates
- ✅ Stable control without oscillations
- ⚠️ Occasional marker misdetection under poor lighting
- ⚠️ Position accuracy degrades when no markers visible

### Task 2: LIDAR Navigation Results

**Test Configuration:**
- Goal: (1.8, -0.6)
- Dynamic obstacles: Cardboard boxes, chairs
- LIDAR range: 1.5m
- Obstacle inflation: ±20°

**Performance Metrics:**

| Metric | Value |
|--------|-------|
| Success rate (reached goal) | 90% |
| Average path length | 2.3m (direct: 2.0m) |
| Path efficiency | 87% |
| Obstacle detection range | 0.1 - 1.5m |
| LIDAR scan rate | 5 Hz |
| Minimum clearance maintained | > 30cm |

**Observations:**
- ✅ Successful obstacle avoidance in cluttered environments
- ✅ Dynamic re-planning when obstacles move
- ✅ No collisions in 18/20 test runs
- ⚠️ Occasional local minima (resolved by manual nudge)
- ⚠️ Path not globally optimal (expected for local planner)

### Localization Accuracy Analysis

**Kalman Filter Performance:**

Comparison of raw ArUco measurements vs. Kalman-filtered estimates:

```
Position RMS Error:
  Raw ArUco:      12.3 cm
  After Kalman:    8.7 cm
  Improvement:    29%

Heading RMS Error:
  Raw ArUco:      7.2°
  After Kalman:   4.1°
  Improvement:    43%
```

**Effect of Process Noise Tuning:**
- Higher Q → More trust in measurements → Faster convergence
- Lower Q → More trust in model → Smoother estimates
- Optimal: Q_v = 0.04 (tuned experimentally)

---

## Performance Analysis

### Computational Performance

**Raspberry Pi 4:**
| Task | CPU Usage | Processing Time |
|------|-----------|-----------------|
| ArUco detection | ~35% | ~40-60 ms/frame |
| LIDAR processing | ~15% | ~20 ms/scan |
| Serial communication | <5% | <1 ms |

**Arduino Uno R3:**
| Task | Execution Time |
|------|----------------|
| Kalman prediction | ~5 ms |
| Kalman update (1 marker) | ~8 ms |
| Motor control update | <1 ms |
| Total loop time | ~15 ms (67 Hz) |

### Limitations & Future Work

**Current Limitations:**

1. **Marker Dependency**: Requires pre-placed markers (not fully autonomous)
2. **Indoor Only**: No GPS, relies on controlled environment
3. **2D Navigation**: No elevation/slope handling
4. **Local Planner**: LIDAR algorithm can get stuck in local minima
5. **Open-Loop Velocity**: No wheel encoders (relies on IMU)

**Proposed Improvements:**

1. **Full SLAM**: Eliminate marker requirement using ORB-SLAM or Cartographer
2. **Global Path Planning**: Implement A* or RRT for optimal paths
3. **Wheel Encoders**: Add for better odometry
4. **Extended Kalman Filter**: Handle nonlinear motion models
5. **Multi-Robot Coordination**: Fleet management
6. **Machine Learning**: Deep learning for improved marker detection under varying lighting

---

## References

See [main report (docs/report.pdf)](docs/report.pdf) for complete bibliography.

Key papers:
1. Garrido-Jurado et al. (2014) - ArUco marker detection
2. Welch & Bishop (2006) - Kalman filter tutorial
3. Fox et al. (1997) - Dynamic Window Approach
4. Thrun et al. (2005) - Probabilistic Robotics

---

For implementation details, see source code and inline documentation.
