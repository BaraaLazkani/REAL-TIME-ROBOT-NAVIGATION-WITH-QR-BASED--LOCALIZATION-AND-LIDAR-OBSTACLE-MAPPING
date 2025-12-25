"""ArUco marker detection and pose estimation."""

import cv2 as cv
import numpy as np
from typing import List, Tuple, Optional


class ArucoDetector:
    """ArUco marker detection and pose estimation using OpenCV."""

    def __init__(self, camera_matrix, dist_coeffs, marker_size,
                 validation_params, dictionary_name="DICT_4X4_50"):
        """Initialize ArUco detector.

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: 1x5 distortion coefficients
            marker_size: Physical marker size in meters
            validation_params: Dictionary with validation thresholds
            dictionary_name: ArUco dictionary name (e.g., "DICT_4X4_50")
        """
        self.mtx = camera_matrix
        self.dist = dist_coeffs
        self.marker_size = marker_size
        self.validation = validation_params

        # Initialize ArUco detector
        dict_id = getattr(cv.aruco, dictionary_name)
        dictionary = cv.aruco.getPredefinedDictionary(dict_id)
        self.detector = cv.aruco.ArucoDetector(dictionary)

        # Object points for marker corners in marker frame
        # Corners: (0,0,0), (s,0,0), (s,s,0), (0,s,0)
        s = marker_size
        self.object_points = np.array([
            [0, 0, 0],
            [s, 0, 0],
            [s, s, 0],
            [0, s, 0]
        ], dtype=np.float32)

    def detect_markers(self, frame: np.ndarray) -> Tuple[List, List, np.ndarray]:
        """Detect ArUco markers in frame.

        Args:
            frame: Input image in BGR format

        Returns:
            Tuple of (marker_corners, marker_ids, annotated_frame)
            - marker_corners: List of detected marker corners
            - marker_ids: List of marker IDs
            - annotated_frame: Frame with markers drawn
        """
        marker_corners, marker_ids, _ = self.detector.detectMarkers(frame)

        # Draw detected markers on frame
        if marker_ids is not None:
            frame = cv.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        return marker_corners, marker_ids, frame

    def estimate_pose(self, corners: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Estimate 6-DOF pose of a detected marker.

        Uses solvePnPRansac for robust pose estimation.

        Args:
            corners: 4x2 array of marker corner coordinates in image

        Returns:
            Tuple of (rvecs, tvecs) if successful, None if pose estimation fails
            - rvecs: 3x1 rotation vector
            - tvecs: 3x1 translation vector
        """
        retval, rvecs, tvecs, _ = cv.solvePnPRansac(
            self.object_points,
            corners[0],
            self.mtx,
            distCoeffs=self.dist,
            flags=cv.SOLVEPNP_IPPE_SQUARE
        )

        if not retval:
            return None

        # Validate detection using reprojection error
        if not self._validate_detection(corners[0], rvecs, tvecs):
            return None

        return rvecs, tvecs

    def _validate_detection(self, corners: np.ndarray, rvecs, tvecs) -> bool:
        """Validate detection quality using area and reprojection error.

        Args:
            corners: Detected marker corners
            rvecs: Rotation vector from pose estimation
            tvecs: Translation vector from pose estimation

        Returns:
            True if detection passes validation, False otherwise
        """
        # Calculate marker area in image (pixels²)
        c = corners
        h = c[0][0] - c[1][0]
        w = c[1][1] - c[2][1]
        area = abs(h * w)

        # Compute reprojection error
        proj, _ = cv.projectPoints(self.object_points, rvecs, tvecs,
                                    self.mtx, self.dist)
        errors = np.linalg.norm(corners - proj.squeeze(), axis=1)
        mean_error = np.mean(errors)

        # Validation thresholds depend on marker area
        # Smaller markers (farther away) have higher error tolerance
        v = self.validation

        if area < v.get('area_threshold_low', 0):
            # Marker too small/far
            return False

        if area < v.get('area_threshold_high', 7100):
            # Small marker: use low area threshold
            return mean_error <= v.get('error_threshold_low_area', 1.4)
        else:
            # Large marker: use high area threshold
            return mean_error <= v.get('error_threshold_high_area', 4.5)

    def get_marker_size(self) -> float:
        """Get configured marker size.

        Returns:
            Marker size in meters
        """
        return self.marker_size
