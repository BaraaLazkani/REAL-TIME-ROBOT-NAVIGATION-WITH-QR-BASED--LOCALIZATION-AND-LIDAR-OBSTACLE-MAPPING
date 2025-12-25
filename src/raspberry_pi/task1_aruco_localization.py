#!/usr/bin/env python3
"""Task 1: ArUco Marker-Based Robot Localization

This script implements visual localization using ArUco markers for
waypoint navigation. The robot estimates its pose by detecting markers
with known world coordinates and fusing with IMU data via Kalman filter
on the Arduino.
"""

import cv2 as cv
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from common.config_loader import get_camera_config, get_robot_config, load_config, get_serial_config
from common.camera import RobotCamera
from common.aruco_detector import ArucoDetector
from common.serial_comm import ArduinoSerial


def transform_to_robot_frame(rvecs, tvecs, Tbc):
    """Transform marker pose from camera frame to robot body frame.

    Uses the camera-to-body transformation matrix to convert detected
    marker pose into robot's coordinate system.

    Args:
        rvecs: 3x1 rotation vector from solvePnP
        tvecs: 3x1 translation vector from solvePnP
        Tbc: 4x4 transformation matrix (camera to body)

    Returns:
        Tuple of (x, y, theta) in robot frame
    """
    # Convert rotation vector to rotation matrix
    R, _ = cv.Rodrigues(rvecs)

    # Build 4x4 transformation matrix (camera to tag)
    Tct = np.eye(4)
    Tct[0:3, 0:3] = R
    Tct[0:3, 3] = tvecs.flatten()

    # Transform to robot frame: body ← camera ← tag
    T = Tbc @ Tct

    # Extract position and orientation
    x = T[0, 3]
    y = T[1, 3]
    theta = np.arctan2(T[1, 0], T[0, 0])

    return x, y, theta


def main():
    """Main execution loop for Task 1."""
    print("=" * 60)
    print("Task 1: ArUco Marker-Based Localization")
    print("=" * 60)

    # Load all configurations
    print("\nLoading configurations...")
    camera_cfg = get_camera_config()
    robot_cfg = get_robot_config()
    task_cfg = load_config('task1_config')
    serial_cfg = get_serial_config('task1')

    print(f"✓ Task: {task_cfg['task_name']}")
    print(f"✓ ArUco Dictionary: {task_cfg['aruco']['dictionary']}")
    print(f"✓ Marker Size: {task_cfg['aruco']['marker_size']}m")
    print(f"✓ Waypoints: {len(task_cfg['waypoints'])}")
    print(f"✓ Serial Port: {serial_cfg['arduino_port']} @ {serial_cfg['baud_rate']} baud")

    # Initialize hardware components
    print("\nInitializing hardware...")
    camera = RobotCamera(camera_cfg)
    print(f"✓ Camera: {camera.get_resolution()}")

    detector = ArucoDetector(
        camera.get_camera_matrix(),
        camera.get_distortion_coeffs(),
        task_cfg['aruco']['marker_size'],
        task_cfg['aruco']['validation'],
        task_cfg['aruco']['dictionary']
    )
    print(f"✓ ArUco Detector initialized")

    arduino = ArduinoSerial(
        serial_cfg['arduino_port'],
        serial_cfg['baud_rate'],
        serial_cfg['timeout']
    )
    print(f"✓ Arduino connected")

    # Get camera-to-body transformation
    Tbc = np.array(robot_cfg['camera_to_body_transform']['matrix'])

    print("\n" + "=" * 60)
    print("Starting navigation... (Press ESC to stop)")
    print("=" * 60 + "\n")

    frame_count = 0

    try:
        while True:
            # Read status from Arduino
            arduino_msg = arduino.readline()
            if arduino_msg:
                print(f"Arduino: {arduino_msg}")

            # Capture frame from camera
            frame = camera.capture_frame()
            frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

            # Detect ArUco markers
            marker_corners, marker_ids, frame_bgr = detector.detect_markers(frame_bgr)

            # Process detected markers
            if marker_ids is not None:
                detections = []

                for i, marker_id in enumerate(marker_ids):
                    # Estimate 6-DOF pose
                    pose = detector.estimate_pose(marker_corners[i])

                    if pose is None:
                        # Failed validation
                        continue

                    rvecs, tvecs = pose

                    # Transform to robot frame
                    x, y, theta = transform_to_robot_frame(rvecs, tvecs, Tbc)

                    detections.append({
                        'x': x,
                        'y': y,
                        'theta': theta,
                        'id': int(marker_id[0])
                    })

                    # Annotate frame
                    cv.putText(
                        frame_bgr,
                        f"ID:{marker_id[0]} ({x:.2f}, {y:.2f})",
                        tuple(marker_corners[i][0][0].astype(int)),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                # Send valid detections to Arduino
                if detections:
                    arduino.send_aruco_data(detections)
                    print(f"Frame {frame_count}: Sent {len(detections)} marker(s)")

            # Display annotated frame
            cv.imshow('Task 1: ArUco Localization', frame_bgr)

            # Check for ESC key
            if cv.waitKey(1) == 27:
                print("\nESC pressed. Stopping...")
                break

            frame_count += 1

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping...")

    finally:
        # Cleanup
        print("\nCleaning up...")
        cv.destroyAllWindows()
        camera.stop()
        arduino.close()
        print("✓ Task 1 stopped successfully")


if __name__ == "__main__":
    main()
