# System Architecture

## Overview Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ROBOT SYSTEM                                 │
│                                                                      │
│  ┌────────────────────────────┐   ┌─────────────────────────────┐  │
│  │     RASPBERRY PI 4         │   │      ARDUINO UNO R3         │  │
│  │  (High-Level Perception)   │   │   (Low-Level Control)       │  │
│  │                            │   │                             │  │
│  │  ┌──────────────────────┐  │   │  ┌────────────────────────┐ │  │
│  │  │  ArUco Detection     │  │   │  │  Kalman Filter         │ │  │
│  │  │  • Marker ID         │  │   │  │  • State: [x, y, θ]    │ │  │
│  │  │  • Pose (R, t)       │  │   │  │  • Prediction step     │ │  │
│  │  │  • Validation        │  │   │  │  • Update step         │ │  │
│  │  └──────────────────────┘  │   │  └────────────────────────┘ │  │
│  │           │                │   │            ▲                │  │
│  │           ▼                │   │            │                │  │
│  │  ┌──────────────────────┐  │   │  ┌────────────────────────┐ │  │
│  │  │  Coordinate Transform│  │   │  │  Navigation Controller │ │  │
│  │  │  • Camera → Body     │  │   │  │  • Proportional        │ │  │
│  │  │  • Body → World      │  │   │  │  • Goal: (x_g, y_g)    │ │  │
│  │  └──────────────────────┘  │   │  │  • Output: (v, ω)      │ │  │
│  │           │                │   │  └────────────────────────┘ │  │
│  │           ▼                │   │            │                │  │
│  │  ┌──────────────────────┐  │   │            ▼                │  │
│  │  │  LIDAR Processing    │  │   │  ┌────────────────────────┐ │  │
│  │  │  • Obstacle detect   │  │   │  │  Motor Control         │ │  │
│  │  │  • Free sectors      │  │   │  │  • Diff drive          │ │  │
│  │  │  • Inflation         │  │   │  │  • PWM output          │ │  │
│  │  └──────────────────────┘  │   │  └────────────────────────┘ │  │
│  │           │                │   │            │                │  │
│  │           ▼                │   │            ▼                │  │
│  │  ┌──────────────────────┐  │   │  ┌────────────────────────┐ │  │
│  │  │  Serial Communication│◄─┼───┼─►│  Serial Parsing        │ │  │
│  │  │  • ArUco: x,y,θ,id   │  │   │  │  • IMU Integration     │ │  │
│  │  │  • LIDAR: sectors    │  │   │  └────────────────────────┘ │  │
│  │  └──────────────────────┘  │   │                             │  │
│  └────────────────────────────┘   └─────────────────────────────┘  │
│         │         │                        │          │            │
│         │         │                        │          │            │
│    ┌────▼───┐ ┌──▼──────┐           ┌─────▼────┐ ┌──▼──────┐     │
│    │Arducam│ │ RPLIDAR │           │ MPU6050  │ │  L298   │     │
│    │ 16MP  │ │  A1M8   │           │   IMU    │ │ Driver  │     │
│    └───────┘ └─────────┘           └──────────┘ └────┬────┘     │
│                                                        │           │
│                                                   ┌────▼────┐     │
│                                                   │ DC      │     │
│                                                   │ Motors  │     │
│                                                   └─────────┘     │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagrams

### Task 1: ArUco Localization

```
┌──────────┐
│  Camera  │
│ (16 MP)  │
└─────┬────┘
      │ RGB Frame
      ▼
┌─────────────────┐
│ ArUco Detector  │
│ • Detect markers│
│ • Solve PnP     │
│ • Validate      │
└─────┬───────────┘
      │ (R, t, ID)
      ▼
┌─────────────────┐
│ Frame Transform │
│ Camera → Body   │
│ Body → World    │
└─────┬───────────┘
      │ (x, y, θ, ID)
      ▼
┌─────────────────┐
│ Serial TX       │──────────┐
│ "2x0.5y1.3...#" │          │
└─────────────────┘          │ USB Serial
                              │ 9600 baud
                              ▼
                    ┌─────────────────┐
                    │ Serial Parser   │
                    │ Extract x,y,θ,ID│
                    └─────┬───────────┘
                          │ Observations
                          ▼
                    ┌─────────────────┐     ┌──────────┐
                    │ Kalman Filter   │◄────┤   IMU    │
                    │ Prediction      │     │ (Angle)  │
                    │ Update          │     └──────────┘
                    └─────┬───────────┘
                          │ State (x,y,θ)
                          ▼
                    ┌─────────────────┐     ┌──────────┐
                    │ Nav Controller  │◄────┤Waypoint  │
                    │ Compute (v, ω)  │     │  Goal    │
                    └─────┬───────────┘     └──────────┘
                          │ (v, ω)
                          ▼
                    ┌─────────────────┐
                    │ Inverse Kinem.  │
                    │ (v,ω)→(v_L,v_R) │
                    └─────┬───────────┘
                          │ PWM
                          ▼
                    ┌─────────────────┐
                    │ Motor Driver    │
                    │ L298 H-Bridge   │
                    └─────┬───────────┘
                          │
                          ▼
                    ┌─────────────────┐
                    │   DC Motors     │
                    │   (Wheels)      │
                    └─────────────────┘
```

### Task 2: LIDAR Navigation

```
┌──────────┐                   ┌──────────┐
│  Camera  │                   │  LIDAR   │
│ (16 MP)  │                   │  A1M8    │
└─────┬────┘                   └─────┬────┘
      │                              │ 360° Scan
      │ (every 3rd cycle)            │
      ▼                              ▼
┌─────────────────┐          ┌─────────────────┐
│ ArUco Detection │          │ LIDAR Processor │
│ (x, y, θ, ID)   │          │ • Segment 5°    │
└─────┬───────────┘          │ • Detect obs    │
      │                      │ • Inflate ±20°  │
      │                      │ • Find free     │
      │                      └─────┬───────────┘
      │                            │ Free sectors
      │                            │
      ├────────────────────────────┤
      │                            │
      ▼                            ▼
┌──────────────────────────────────────────┐
│         Serial Communication             │
│  ArUco: "1x0.5y1.3t0.9i7#"              │
│  LIDAR: "s0s5s10s15...s355#"            │
└─────────────────┬────────────────────────┘
                  │ USB 115200 baud
                  ▼
┌──────────────────────────────────────────┐
│            Arduino Parser                │
│  • Parse ArUco observations              │
│  • Parse free sector list                │
└─────┬────────────────────┬───────────────┘
      │                    │
      │ Observations       │ Free sectors
      ▼                    │
┌─────────────────┐        │
│ Kalman Filter   │◄───────┼────┐
│ (x, y, θ)       │        │    │ IMU
└─────┬───────────┘        │    │
      │ Current pose       │    │
      │                    ▼    │
      │              ┌──────────────────┐
      │              │ Dynamic Window   │
      │              │ • Find best dir  │
      │              │ • Set local goal │
      │              └─────┬────────────┘
      │                    │ Local goal
      ▼                    │
┌──────────────────────────▼───────────────┐
│         Navigation Controller            │
│  Compute (v, ω) toward local goal       │
│  IF no LIDAR: STOP (safety)             │
└─────────────────┬────────────────────────┘
                  │ (v, ω)
                  ▼
            [Motor Control]
                  │
                  ▼
              [Wheels]
```

## State Machine Diagrams

### Task 1: Waypoint Navigation State Machine

```
                   ┌─────────┐
                   │  START  │
                   └────┬────┘
                        │
                        ▼
                 ┌──────────────┐
            ┌───►│  NAVIGATING  │
            │    │  to waypoint │
            │    │      N       │
            │    └──────┬───────┘
            │           │
            │           │ dist < threshold
            │           ▼
            │    ┌──────────────┐
            │    │   WAYPOINT   │
            │    │   REACHED    │
            │    └──────┬───────┘
            │           │
            │           │ N++
            │           │
            └───────────┤
                        │
                        │ N >= total waypoints
                        ▼
                 ┌──────────────┐
                 │  GOAL        │
                 │  REACHED     │
                 │  (STOP)      │
                 └──────────────┘

States:
  NAVIGATING:
    - Read ArUco markers
    - Update Kalman filter
    - Compute velocity command
    - Drive motors

  WAYPOINT REACHED:
    - Increment waypoint index
    - Brief pause (optional)
    - Check if final goal

  GOAL REACHED:
    - Stop motors
    - Terminate program
```

### Task 2: LIDAR Navigation State Machine

```
                   ┌─────────┐
                   │  START  │
                   └────┬────┘
                        │
                        ▼
                 ┌──────────────┐
            ┌───►│ WAITING FOR  │
            │    │    LIDAR     │
            │    └──────┬───────┘
            │           │
            │           │ LIDAR ready
            │           ▼
            │    ┌──────────────┐
            ├───►│ NAVIGATING   │◄────────┐
            │    │ to local goal│         │
            │    └──────┬───────┘         │
            │           │                 │
            │           │ goal reached    │
            │           ▼                 │
            │    ┌──────────────┐         │
            │    │  UPDATE      │         │
            │    │ LOCAL GOAL   │─────────┘
            │    └──────┬───────┘
            │           │
            │           │ dist to final < threshold
            │           ▼
            └─── ┌──────────────┐
                 │ FINAL GOAL   │
                 │   REACHED    │
                 │   (STOP)     │
                 └──────────────┘

States:
  WAITING FOR LIDAR:
    - Motors stopped
    - Wait for first LIDAR scan
    - Safety measure

  NAVIGATING:
    - Process LIDAR scans
    - Update Kalman with ArUco
    - Navigate to local goal
    - Avoid obstacles

  UPDATE LOCAL GOAL:
    - Compute free sectors
    - Select best direction
    - Set new local goal (0.8m away)
    - Transition timer (3s)

  FINAL GOAL REACHED:
    - Stop motors
    - Terminate
```

## Module Dependency Graph

### Python Modules (Raspberry Pi)

```
task1_aruco_localization.py
    │
    ├─► common/config_loader.py ──► PyYAML
    │
    ├─► common/camera.py ──────────► Picamera2
    │                               └─► NumPy
    │
    ├─► common/aruco_detector.py ──► OpenCV
    │                               └─► NumPy
    │
    └─► common/serial_comm.py ─────► PySerial


task2_slam_navigation.py
    │
    ├─► common/config_loader.py ──► PyYAML
    │
    ├─► common/camera.py ──────────► Picamera2
    │
    ├─► common/aruco_detector.py ──► OpenCV
    │
    ├─► common/serial_comm.py ─────► PySerial
    │
    └─► lidar_processor.py ────────► RPLidar SDK
                                    └─► Threading, Queue
```

### Arduino Modules

```
task1_aruco_nav.ino
    │
    ├─► common/kalman_filter.h ────► BasicLinearAlgebra
    │
    ├─► common/motor_control.h
    │
    ├─► common/navigation.h
    │
    └─► MPU6050_light ─────────────► Wire.h (I2C)


task2_slam_nav.ino
    │
    ├─► common/kalman_filter.h ────► BasicLinearAlgebra
    │
    ├─► common/motor_control.h
    │
    ├─► common/navigation.h
    │
    └─► MPU6050_light ─────────────► Wire.h (I2C)
```

## Communication Protocol

### Serial Message Format

**Raspberry Pi → Arduino**

```
ArUco Message:
┌──────┬────────────────────────────────────────┬──┐
│Count │  x,y,θ,id (repeated for each marker)  │ #│
└──────┴────────────────────────────────────────┴──┘
Format: "{count}x{x}y{y}t{theta}i{id}x...#"

Example: "2x0.523y1.342t0.981i7x0.614y1.285t1.023i9#"

LIDAR Message:
┌──┬────────────────────────────────────┬──┐
│ s│  angle s angle s angle s ...       │ #│
└──┴────────────────────────────────────┴──┘
Format: "s{angle}s{angle}s...#"

Example: "s0s5s10s15s20s340s345s350s355#"
```

**Arduino → Raspberry Pi**

```
Status/Debug Messages:
"Task 1 Arduino Ready"
"Waypoint 1 reached"
"0.523 1.342 0.981"  (x y theta state)
"Goal reached!"
```

### Timing Diagram

```
Raspberry Pi:                Arduino:
    │                            │
    │────"START\n"──────────────►│
    │                            │ Initialize
    │                            │ IMU calibration
    │◄───"Ready"─────────────────│
    │                            │
    ├─ Capture frame             │
    ├─ Detect ArUco              │
    │────"2x0.5y1.3...#"────────►│─ Parse
    │                            ├─ Kalman predict
    │                            ├─ Kalman update
    │◄───"0.5 1.3 0.9"───────────│─ Print state
    │                            ├─ Nav controller
    │                            └─ Motor control
    │                            │
    │◄── (10ms later) ───────────│
    │                            │
    ├─ LIDAR scan                │
    │────"s0s5s10...#"──────────►│─ Parse sectors
    │                            ├─ Update local goal
    │                            │
    ├─ Capture frame             │
    │  (every 3rd cycle)         │
    │────"1x0.6y1.4...#"────────►│
    │                            │
    └─ Loop @10Hz                └─ Loop @67Hz
```

## Hardware Interfacing

### Pin Connections

**Arduino Uno R3:**

```
┌─────────────────────────────────────┐
│           DIGITAL PINS              │
├─────┬───────────────────────────────┤
│ 5   │ Motor Left Enable (PWM)       │
│ 6   │ Motor Left IN2                │
│ 7   │ Motor Left IN1                │
│ 8   │ Motor Right IN3               │
│ 9   │ Motor Right IN4               │
│ 10  │ Motor Right Enable (PWM)      │
└─────┴───────────────────────────────┘

┌─────────────────────────────────────┐
│           I2C (ANALOG)              │
├─────┬───────────────────────────────┤
│ A4  │ SDA (MPU6050)                 │
│ A5  │ SCL (MPU6050)                 │
└─────┴───────────────────────────────┘

┌─────────────────────────────────────┐
│              SERIAL                 │
├─────┬───────────────────────────────┤
│ 0   │ RX (from Raspberry Pi)        │
│ 1   │ TX (to Raspberry Pi)          │
└─────┴───────────────────────────────┘
```

**Raspberry Pi 4:**

```
┌─────────────────────────────────────┐
│            INTERFACES               │
├─────────────┬───────────────────────┤
│ CSI         │ Arducam 16MP          │
│ USB 2.0 #1  │ Arduino (Serial)      │
│ USB 2.0 #2  │ RPLIDAR A1M8          │
│ (I2C unused)│ (IMU on Arduino)      │
└─────────────┴───────────────────────┘
```

### Power Distribution

```
┌──────────────────────────────────────────┐
│         12V Battery / Power Supply       │
└────────────┬─────────────────────────────┘
             │
      ┌──────┴──────┐
      │             │
      ▼             ▼
┌───────────┐ ┌─────────────┐
│  5V Buck  │ │ L298 Motor  │
│ Converter │ │   Driver    │
└─────┬─────┘ └──────┬──────┘
      │              │
      ├──────────────┼──────────┐
      │              │          │
      ▼              ▼          ▼
┌──────────┐  ┌──────────┐ ┌────────┐
│Raspberry │  │ Arduino  │ │ DC     │
│  Pi 4    │  │ Uno R3   │ │ Motors │
└──────────┘  └──────────┘ └────────┘
      │              │
      ▼              ▼
┌──────────┐  ┌──────────┐
│ Arducam  │  │ MPU6050  │
│ RPLIDAR  │  │          │
└──────────┘  └──────────┘
```

---

## Performance Characteristics

### Latency Budget

```
Camera Capture:        40-60 ms
ArUco Detection:       30-50 ms
Coordinate Transform:   < 1 ms
Serial TX:             < 1 ms
─────────────────────────────────
Total (Pi):            70-111 ms  (9-14 Hz)

Serial Parse:           1-2 ms
Kalman Filter:          8-13 ms
Navigation:             < 1 ms
Motor Control:          < 1 ms
─────────────────────────────────
Total (Arduino):       10-17 ms   (58-100 Hz)

End-to-End Latency:    80-128 ms  (8-12 Hz)
```

### Memory Usage

**Raspberry Pi 4 (Python):**
- Base process: ~50 MB
- OpenCV: ~120 MB
- Camera buffer: ~10 MB
- Total: ~180 MB

**Arduino Uno R3:**
- Program: ~18 KB / 32 KB Flash (56%)
- Variables: ~1.2 KB / 2 KB RAM (60%)
- Stack: ~400 bytes

---

For implementation details, see `TECHNICAL.md` and source code.
