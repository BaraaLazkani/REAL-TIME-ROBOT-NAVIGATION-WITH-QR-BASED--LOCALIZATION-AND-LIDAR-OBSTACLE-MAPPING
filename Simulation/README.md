# Robot Navigation Simulation

This directory contains a Python-based simulation environment for testing and visualizing autonomous robot navigation algorithms before deploying to hardware. The simulation implements the complete navigation stack including path planning, state estimation, and motion control.

**Watch the simulation in action:** [YouTube Demo](https://youtu.be/W2GyEFLIgFA)

## 📋 Overview

The simulation provides a virtual testbed for validating navigation algorithms in a controlled environment with simulated sensors and obstacles. It implements:

- **Path Planning**: Dijkstra's algorithm for optimal obstacle-free paths
- **State Estimation**: Kalman filter for sensor fusion (camera + IMU)
- **Motion Control**: Differential drive proportional controller
- **Real-time Visualization**: Animated display of robot motion and state estimates

This simulation environment was used to develop and validate the algorithms before implementation on the physical robot described in the main project.

## 🗂️ Directory Structure

```
Simulation/
├── src/                    # Source code
│   ├── RobotControl.py           # Main simulation controller
│   ├── RobotSim.py               # Simulation environment & visualization
│   ├── KalmanFilter.py           # State estimation implementation
│   ├── DiffDriveController.py    # Differential drive controller
│   ├── shortestpath.py           # Dijkstra's path planning
│   ├── utility.py                # Helper functions
│   └── params.yaml               # Simulation configuration
├── plots/                  # Generated visualizations
│   ├── simulation_environment.png      # Map layout with obstacles & markers
│   ├── simulation_path_planning.png    # Dijkstra's computed path
│   ├── simulation_trajectory.png       # Ground truth vs Kalman estimates
│   ├── simulation_controller.png       # Control signals & performance
│   ├── simulation_sensor_fusion.png    # Sensor fusion analysis
│   ├── simulation_error_analysis.png   # Error metrics & distributions
│   └── simulation_path_scenarios.png   # Multiple navigation scenarios
└── README.md               # This file
```

## 🎨 Visualizations

The `plots/` directory contains seven publication-quality visualizations demonstrating different aspects of the navigation system:

### 1. Environment Map (`simulation_environment.png`)
- Occupancy grid showing obstacle layout
- ArUco marker positions with IDs
- Robot start and goal positions
- Gridded metric coordinates

### 2. Path Planning (`simulation_path_planning.png`)
- Dijkstra's algorithm shortest path computation
- Obstacle avoidance demonstration
- Waypoint progression from start to goal

### 3. Trajectory Analysis (`simulation_trajectory.png`)
- Ground truth robot position vs Kalman filter estimates
- Position error over time
- Performance statistics (mean error, RMS error, max error)

### 4. Controller Analysis (`simulation_controller.png`)
- Distance to goal convergence
- Linear velocity commands vs actual velocity
- Angular velocity profile
- Total control effort metrics

### 5. Sensor Fusion (`simulation_sensor_fusion.png`)
- X/Y position: Raw ArUco detections vs Kalman filtered estimates
- Orientation fusion: IMU + ArUco → Kalman output
- Kalman gain evolution showing filter convergence

### 6. Error Metrics (`simulation_error_analysis.png`)
- Position error comparison: raw measurements vs filtered
- Cumulative error accumulation
- Error distribution histograms
- Performance improvement metrics

### 7. Path Scenarios (`simulation_path_scenarios.png`)
Four navigation scenarios demonstrating algorithm robustness:
- Direct navigation (obstacle-free)
- Single obstacle avoidance
- Narrow passage navigation
- Complex multi-obstacle environment

## 🧮 Algorithms Implemented

### 1. Dijkstra's Path Planning
**File**: `src/shortestpath.py`

Computes the shortest collision-free path from start to goal:
```python
goals = dijkstras(occupancy_map, x_spacing, y_spacing, pos_init, pos_goal)
```

- Discretizes continuous environment into a grid
- Uses priority queue for efficient distance computation
- Backtracks from goal to start to extract optimal path
- Returns waypoint sequence for controller to follow

### 2. Kalman Filter
**File**: `src/KalmanFilter.py`

Fuses noisy camera (ArUco) and IMU measurements for optimal state estimation:

**Prediction Step:**
```
x_k = x_{k-1} + v*dt*cos(θ)
y_k = y_{k-1} + v*dt*sin(θ)
θ_k = ω_IMU
```

**Update Step (when ArUco detected):**
```
K = P * H^T * (H * P * H^T + R)^{-1}      # Kalman gain
x = x + K * (z - H*x)                      # State update
P = (I - K*H) * P                          # Covariance update
```

Provides smoother, more accurate position estimates than raw sensor measurements.

### 3. Differential Drive Controller
**File**: `src/DiffDriveController.py`

Proportional controller for waypoint navigation:

```python
ρ = sqrt((x_goal - x)^2 + (y_goal - y)^2)      # Distance to goal
α = -θ + atan2(y_goal - y, x_goal - x)          # Heading error
β = -θ - α                                       # Final orientation error

v = k_p * ρ                                      # Linear velocity
ω = k_a * α + k_b * β                            # Angular velocity
```

**Tuning Parameters:**
- `k_p = 0.5`: Distance gain
- `k_a = 1.0`: Heading gain
- `k_b = 0.0`: Final orientation gain (not used in simulation)

## ⚙️ Configuration

The simulation is configured via `src/params.yaml`:

```yaml
# Occupancy grid (0 = free, 1 = obstacle)
occupancy_map:
  - [0, 0, 0, 0, 0, 0, 0, 0]
  - [1, 1, 1, 1, 1, 1, 1, 1]
  # ... 10 rows total (10x8 grid)

# ArUco marker database [x, y, theta, id]
world_map:
  - [0.230, 0., -1.570796, 0.0]
  - [0.383, 0., -1.570796, 1.0]
  # ... 12 markers total

# Robot initial state [x, y, theta]
pos_init:
  - [0.5]
  - [1.0]
  - [1.5707963267948966]  # 90 degrees

# Robot goal state [x, y, theta]
pos_goal:
  - [1.1]
  - [0.9]
  - [-1.5707963267948966]  # -90 degrees

# Robot constraints
max_vel: 1.0              # m/s
max_omega: 6.28318        # rad/s (2π)
x_spacing: 0.2            # Grid cell width (m)
y_spacing: 0.2            # Grid cell height (m)
```

**To modify the simulation:**
1. Edit obstacle layout in `occupancy_map`
2. Update marker positions in `world_map`
3. Change start/goal positions
4. Adjust robot velocity limits

## 🚀 Running the Simulation

### Prerequisites

```bash
pip install numpy matplotlib pyyaml
```

**Note**: The simulation uses older Python/NumPy APIs and works best with:
- Python 3.7-3.10 (not 3.11+)
- NumPy 1.19.x - 1.21.x
- Matplotlib 3.3.x - 3.5.x

### Running the Interactive Simulation

```bash
cd Simulation/src
python RobotControl.py
```

This opens a real-time visualization window showing:
- **Red rectangles**: Obstacles
- **Yellow rectangles**: ArUco markers with IDs
- **Blue robot**: Ground truth position
- **Black outline**: Kalman filter estimate
- **Black line**: Trajectory trail
- **Green markers**: Currently visible markers

The simulation runs until the robot reaches the goal or the window is closed.

### Generating Static Plots

If you encounter compatibility issues or prefer static visualizations, you can generate the plots without running the full simulation (plots are pre-generated in `plots/` directory).

## 📊 Simulation Results

The simulation demonstrates:
- **Path Planning**: Successfully navigates around obstacles using Dijkstra's algorithm
- **State Estimation**: Kalman filter reduces position error by ~30% compared to raw measurements
- **Control Performance**: Smooth convergence to goal with proportional control

**Key Performance Metrics** (from trajectory analysis):
- Mean position error: ~0.02m
- RMS error: ~0.03m
- Maximum error: ~0.08m

## 🔗 Relation to Hardware Implementation

This simulation formed the algorithmic foundation for the physical robot navigation system:

### Algorithm Transfer
- **Kalman Filter** → Implemented in Arduino (`src/arduino/common/kalman_filter.h`)
- **Differential Drive Control** → Adapted in Arduino (`src/arduino/common/navigation.h`)
- **Path Planning Concept** → Extended to Dynamic Window Approach for real-time obstacle avoidance

### Key Differences: Simulation vs Hardware

| Aspect | Simulation | Hardware |
|--------|------------|----------|
| **Sensors** | Simulated camera + IMU | Arducam 16MP + MPU6050 |
| **Markers** | Pre-placed in virtual world | Physical ArUco tags (96mm) |
| **Obstacles** | Static occupancy grid | Real-time LIDAR scanning (RPLIDAR A1M8) |
| **Noise** | Gaussian noise models | Real sensor noise + calibration errors |
| **Platform** | Python (matplotlib visualization) | Raspberry Pi 4 + Arduino Uno R3 |
| **Path Planning** | Dijkstra (offline) | Dynamic Window Approach (online) |

The simulation provided a risk-free environment to:
- ✅ Validate Kalman filter tuning (Q and R matrices)
- ✅ Test controller gains (k_p, k_a)
- ✅ Verify path planning logic
- ✅ Analyze state estimation performance
- ✅ Debug algorithms before hardware deployment

## 🔧 Troubleshooting

### "ModuleNotFoundError: No module named 'numpy'"
Install required packages:
```bash
pip install numpy matplotlib pyyaml
```

### "AttributeError: module 'numpy' has no attribute 'float'"
Your NumPy version is too new. The code uses deprecated `np.float` and `np.int` types. Either:
- Downgrade NumPy: `pip install numpy==1.21.6`
- Or use Python 3.10 or earlier

### Display Issues (Headless Environment)
If running without a display:
```bash
export MPLBACKEND=Agg  # Linux/Mac
set MPLBACKEND=Agg     # Windows
```

### Simulation Runs Too Fast/Slow
The simulation speed is controlled by matplotlib's animation refresh rate. If you need to adjust timing, modify the `plt.pause()` value in `RobotSim.py` (typically line ~403).

## 📚 References

- **Dijkstra's Algorithm**: [Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- **Kalman Filter**: [Welch & Bishop, "An Introduction to the Kalman Filter"](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
- **Differential Drive Kinematics**: [Columbia University Notes](https://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf)
- **ArUco Markers**: [OpenCV Documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

---

**For the complete robot navigation system**, see the [main project README](../README.md).

**Watch the simulation demo**: [YouTube](https://youtu.be/W2GyEFLIgFA)
