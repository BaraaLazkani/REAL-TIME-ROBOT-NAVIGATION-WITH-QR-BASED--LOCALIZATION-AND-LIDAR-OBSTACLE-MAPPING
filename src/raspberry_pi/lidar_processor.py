"""LIDAR processing for obstacle detection and avoidance."""

import numpy as np
from rplidar import RPLidar
import queue
import threading
from typing import List, Set


class LidarProcessor:
    """RPLIDAR A1M8 processor for real-time obstacle detection."""

    def __init__(self, port: str, max_dist: float, min_dist: float,
                 sector_width: int, inflation: int):
        """Initialize LIDAR processor.

        Args:
            port: Serial port for LIDAR (e.g., "/dev/ttyUSB0")
            max_dist: Maximum detection distance in meters
            min_dist: Minimum detection distance in meters
            sector_width: Angular resolution in degrees (typically 5°)
            inflation: Obstacle inflation angle in degrees (safety buffer)
        """
        self.port = port
        self.max_dist = max_dist * 1000  # Convert meters to millimeters
        self.min_dist = min_dist
        self.sector_width = sector_width
        self.inflation = inflation

        self.scan_queue = queue.Queue(maxsize=3)
        self.lidar = RPLidar(port, baudrate=115200, timeout=3)
        self.running = False
        self.thread = None

    def start(self):
        """Start LIDAR scanning in background thread."""
        if self.running:
            return

        self.running = True
        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()

    def _scan_loop(self):
        """Background thread for continuous LIDAR scanning.

        Scans are added to queue, replacing old scans if queue is full.
        """
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break

                # Replace old scan if queue is full (keep most recent)
                if self.scan_queue.full():
                    try:
                        self.scan_queue.get_nowait()
                    except queue.Empty:
                        pass

                self.scan_queue.put(scan)
        except Exception as e:
            print(f"LIDAR scan error: {e}")
            self.running = False

    def get_free_sectors(self) -> List[int]:
        """Get list of obstacle-free angular sectors.

        Processes latest LIDAR scan to identify sectors without obstacles
        within the configured distance range. Applies safety inflation
        around detected obstacles.

        Returns:
            List of free sector angles (0-355 in steps of sector_width)
            Empty list if no scan available
        """
        try:
            scan = self.scan_queue.get(timeout=1)
        except queue.Empty:
            return []

        # Identify sectors with obstacles
        obstacle_sectors = self._detect_obstacle_sectors(scan)

        # Apply safety inflation around obstacles
        inflated_sectors = self._inflate_obstacles(obstacle_sectors)

        # Get free sectors (complement of inflated obstacles)
        all_sectors = set(range(0, 360, self.sector_width))
        free_sectors = sorted(all_sectors - inflated_sectors)

        return free_sectors

    def _detect_obstacle_sectors(self, scan) -> Set[int]:
        """Detect sectors containing obstacles within range.

        Args:
            scan: LIDAR scan data (list of (quality, angle, distance) tuples)

        Returns:
            Set of sector angles containing obstacles
        """
        obstacle_sectors = set()

        for quality, angle, dist in scan:
            # Quantize angle to sector
            sector = (int(angle // self.sector_width) * self.sector_width) % 360

            # Check if obstacle is within detection range
            if self.min_dist <= dist <= self.max_dist:
                obstacle_sectors.add(sector)

        return obstacle_sectors

    def _inflate_obstacles(self, obstacle_sectors: Set[int]) -> Set[int]:
        """Apply safety inflation around obstacle sectors.

        Creates a buffer zone around each obstacle by marking adjacent
        sectors as occupied.

        Args:
            obstacle_sectors: Set of sectors with detected obstacles

        Returns:
            Set of sectors including inflated safety zones
        """
        inflated_sectors = set()

        for sector in obstacle_sectors:
            # Inflate by ±inflation degrees
            for offset in range(-self.inflation, self.inflation + 1, self.sector_width):
                inflated = (sector + offset) % 360
                inflated_sectors.add(inflated)

        return inflated_sectors

    def get_latest_scan(self):
        """Get the most recent LIDAR scan.

        Returns:
            Latest scan data or None if no scan available
        """
        try:
            return self.scan_queue.get_nowait()
        except queue.Empty:
            return None

    def is_running(self) -> bool:
        """Check if LIDAR is actively scanning.

        Returns:
            True if scanning, False otherwise
        """
        return self.running

    def stop(self):
        """Stop LIDAR scanning and motor."""
        self.running = False

        # Wait for thread to finish
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        # Stop LIDAR motor and disconnect
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception as e:
            print(f"LIDAR stop error: {e}")

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures LIDAR is stopped."""
        self.stop()
