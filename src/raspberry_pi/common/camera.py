"""Camera interface wrapper for Raspberry Pi Camera."""

import numpy as np
from picamera2 import Picamera2
from typing import Tuple


class RobotCamera:
    """Wrapper for Raspberry Pi camera with calibration parameters."""

    def __init__(self, config: dict):
        """Initialize camera with configuration.

        Args:
            config: Camera configuration dictionary from camera_calibration.yaml
        """
        self.config = config
        self.picam2 = Picamera2()
        self._setup_camera()

    def _setup_camera(self):
        """Configure and start the camera with specified resolution."""
        width = self.config['camera']['resolution']['width']
        height = self.config['camera']['resolution']['height']

        config = self.picam2.create_preview_configuration(
            main={"size": (width, height)}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def get_camera_matrix(self) -> np.ndarray:
        """Get camera intrinsic matrix (K).

        Returns:
            3x3 camera matrix as numpy array
        """
        return np.array(self.config['camera']['matrix'])

    def get_distortion_coeffs(self) -> np.ndarray:
        """Get camera distortion coefficients.

        Returns:
            1x5 distortion coefficient array [k1, k2, p1, p2, k3]
        """
        return np.array([self.config['camera']['distortion_coefficients']])

    def capture_frame(self) -> np.ndarray:
        """Capture a single frame from the camera.

        Returns:
            Frame as numpy array in RGB format
        """
        return self.picam2.capture_array()

    def get_resolution(self) -> Tuple[int, int]:
        """Get configured camera resolution.

        Returns:
            Tuple of (width, height)
        """
        return (
            self.config['camera']['resolution']['width'],
            self.config['camera']['resolution']['height']
        )

    def stop(self):
        """Stop the camera."""
        self.picam2.stop()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures camera is stopped."""
        self.stop()
