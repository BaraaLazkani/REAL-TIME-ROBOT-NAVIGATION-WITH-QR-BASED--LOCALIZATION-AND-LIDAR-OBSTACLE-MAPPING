#!/usr/bin/env python3
"""Task 2: SLAM with LIDAR Navigation

This script implements autonomous navigation with obstacle avoidance using
LIDAR sensor fusion. Combines ArUco visual localization with real-time
LIDAR-based obstacle detection using a Dynamic Window Approach-inspired algorithm.
"""

import cv2 as cv
import numpy as np
import sys
import signal
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from common.config_loader import get_camera_config, get_robot_config, load_config, get_serial_config
from common.camera import RobotCamera
from common.aruco_detector import ArucoDetector
from common.serial_comm import ArduinoSerial
from lidar_processor import LidarProcessor


# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global running
    print("\nShutdown signal received...")
    running = False


def transform_to_robot_frame(rvecs, tvecs, Tbc):
    """Transform marker pose from camera frame to robot body frame.

    Args:
        rvecs: 3x1 rotation vector from solvePnP
        tvecs: 3x1 translation vector from solvePnP
        Tbc: 4x4 transformation matrix (camera to body)

    Returns:
        Tuple of (x, y, theta) in robot frame
    """
    R, _ = cv.Rodrigues(rvecs)

    Tct = np.eye(4)
    Tct[0:3, 0:3] = R
    Tct[0:3, 3] = tvecs.flatten()

    T = Tbc @ Tct

    x = T[0, 3]
    y = T[1, 3]
    theta = np.arctan2(T[1, 0], T[0, 0])

    return x, y, theta


def main():
    """Main execution loop for Task 2."""
    global running

    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60)
    print("Task 2: SLAM with LIDAR Navigation")
    print("=" * 60)

    # Load all configurations
    print("\nLoading configurations...")
    camera_cfg = get_camera_config()
    robot_cfg = get_robot_config()
    task_cfg = load_config('task2_config')
    serial_cfg = get_serial_config('task2')

    print(f"✓ Task: {task_cfg['task_name']}")
    print(f"✓ Goal: ({task_cfg['goal']['x']:.2f}, {task_cfg['goal']['y']:.2f})")
    print(f"✓ LIDAR Max Distance: {task_cfg['lidar']['max_distance']}m")
    print(f"✓ Obstacle Inflation: ±{task_cfg['lidar']['obstacle_inflation']}°")

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

    lidar = LidarProcessor(
        serial_cfg['lidar_port'],
        task_cfg['lidar']['max_distance'],
        task_cfg['lidar']['min_distance'],
        task_cfg['lidar']['sector_width'],
        task_cfg['lidar']['obstacle_inflation']
    )
    lidar.start()
    print(f"✓ LIDAR scanning started on {serial_cfg['lidar_port']}")

    arduino = ArduinoSerial(
        serial_cfg['arduino_port'],
        serial_cfg['arduino_baud'],
        serial_cfg['timeout']
    )
    print(f"✓ Arduino connected @ {serial_cfg['arduino_baud']} baud")

    # Get camera-to-body transformation
    Tbc = np.array(robot_cfg['camera_to_body_transform']['matrix'])

    # Processing control
    frame_count = 0
    aruco_interval = task_cfg['processing']['aruco_update_interval']

    print("\n" + "=" * 60)
    print("Starting navigation... (Press ESC to stop)")
    print("=" * 60 + "\n")

    try:
        while running:
            # Read status from Arduino
            arduino_msg = arduino.readline()
            if arduino_msg:
                print(f"Arduino: {arduino_msg}")

            # Process LIDAR data every cycle
            free_sectors = lidar.get_free_sectors()
            if free_sectors and frame_count >= aruco_interval:
                arduino.send_lidar_sectors(free_sectors)
                print(f"LIDAR: {len(free_sectors)} free sectors")

            # Process ArUco markers at reduced rate
            if frame_count >= aruco_interval:
                frame = camera.capture_frame()
                frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

                marker_corners, marker_ids, frame_bgr = detector.detect_markers(frame_bgr)

                if marker_ids is not None:
                    detections = []

                    # Process only first detected marker for efficiency
                    for i in range(min(1, len(marker_ids))):
                        pose = detector.estimate_pose(marker_corners[i])

                        if pose is None:
                            continue

                        rvecs, tvecs = pose
                        x, y, theta = transform_to_robot_frame(rvecs, tvecs, Tbc)

                        detections.append({
                            'x': x,
                            'y': y,
                            'theta': theta,
                            'id': int(marker_ids[i][0])
                        })

                        # Annotate frame
                        cv.putText(
                            frame_bgr,
                            f"ID:{marker_ids[i][0]} ({x:.2f}, {y:.2f})",
                            tuple(marker_corners[i][0][0].astype(int)),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )

                    # Send to Arduino
                    if detections:
                        arduino.send_aruco_data(detections)
                        print(f"ArUco: Sent marker ID {detections[0]['id']}")

                # Display annotated frame
                cv.imshow('Task 2: SLAM Navigation', frame_bgr)

                # Check for ESC key
                if cv.waitKey(1) == 27:
                    print("\nESC pressed. Stopping...")
                    break

            frame_count += 1
            if frame_count >= aruco_interval:
                frame_count = 0

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping...")

    finally:
        # Cleanup
        print("\nCleaning up...")
        running = False
        lidar.stop()
        cv.destroyAllWindows()
        camera.stop()
        arduino.close()
        print("✓ Task 2 stopped successfully")


if __name__ == "__main__":
    main()
