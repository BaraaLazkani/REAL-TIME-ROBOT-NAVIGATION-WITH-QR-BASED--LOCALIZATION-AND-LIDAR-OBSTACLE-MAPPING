# Camera Calibration

This directory contains tools for calibrating the Raspberry Pi camera used in the robot navigation system.

## Overview

Camera calibration is essential for accurate ArUco marker pose estimation. The calibration process determines:
- **Intrinsic matrix** (focal length, principal point)
- **Distortion coefficients** (radial and tangential distortion)

## Calibration Process

### Step 1: Capture Calibration Images

```bash
python3 capture_images.py
```

- Print a 4×4 chessboard pattern
- Position the chessboard at various distances and angles
- Capture 40-50 images showing the full chessboard
- Images are automatically saved

### Step 2: Run Calibration

```bash
python3 calibrate_camera.py
```

This script:
1. Loads all captured images
2. Detects chessboard corners
3. Computes camera matrix and distortion coefficients
4. Reports reprojection error

### Step 3: Update Configuration

Copy the output camera matrix and distortion coefficients to `config/camera_calibration.yaml`.

## Current Calibration

The current calibration was performed using:
- **Chessboard**: 4×4 grid
- **Square size**: 96mm
- **Images**: 42 successful detections
- **Reprojection error**: 0.310 pixels

### Camera Matrix (K)
```
[[583.40  0.00  317.27]
 [  0.00  568.88  226.75]
 [  0.00    0.00    1.00]]
```

### Distortion Coefficients [k1, k2, p1, p2, k3]
```
[0.4376, -2.5317, -0.0013, 0.0147, 4.6427]
```

## Tips for Good Calibration

1. **Lighting**: Use consistent, diffuse lighting
2. **Coverage**: Capture images with chessboard in all areas of the frame
3. **Angles**: Include tilted views (±30°) not just frontal
4. **Distance**: Vary distance from 20cm to 1m
5. **Focus**: Ensure images are sharp and in focus
6. **Quantity**: 40-50 good images is ideal

## Troubleshooting

**"Chessboard not detected"**
- Ensure the pattern is flat and well-lit
- Check that the entire chessboard is visible
- Try different angles or distances

**High reprojection error (>0.5 pixels)**
- Remove blurry or poorly detected images
- Ensure chessboard dimensions are correct
- Recapture with better image quality

## Files

- `calibrate_camera.py` - Main calibration script
- `capture_images.py` - Image capture utility
- `images/` - Directory for calibration images (create manually)
