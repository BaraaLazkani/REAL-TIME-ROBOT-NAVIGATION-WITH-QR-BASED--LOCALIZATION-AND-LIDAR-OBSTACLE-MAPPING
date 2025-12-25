"""Serial communication with Arduino."""

import serial
import time
from typing import Dict, List, Any


class ArduinoSerial:
    """Serial communication handler for Arduino."""

    def __init__(self, port: str, baud_rate: int, timeout: float = 0.1):
        """Initialize serial connection to Arduino.

        Args:
            port: Serial port path (e.g., "/dev/ttyACM0")
            baud_rate: Communication baud rate (9600 or 115200)
            timeout: Read timeout in seconds
        """
        self.arduino = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
        self.arduino.flush()
        time.sleep(2)  # Wait for Arduino to reset
        self._send_start_signal()

    def _send_start_signal(self):
        """Send START signal to Arduino to begin operation."""
        self.arduino.write(b"START\n")
        time.sleep(1)

    def send_aruco_data(self, detections: List[Dict[str, Any]]):
        """Send ArUco detection data to Arduino.

        Protocol format: {count}x{x}y{y}t{theta}i{id}x...#

        Example: "2x0.52y1.34t0.98i7x0.61y1.28t1.02i9#"

        Args:
            detections: List of detection dictionaries with keys:
                       'x', 'y', 'theta', 'id'
        """
        if not detections:
            return

        # Build message string
        message = str(len(detections))
        for det in detections:
            message += f"x{det['x']:.3f}y{det['y']:.3f}t{det['theta']:.3f}i{det['id']}"
        message += "#"

        # Send to Arduino
        self.arduino.write(message.encode())
        time.sleep(0.1)

    def send_lidar_sectors(self, free_sectors: List[int]):
        """Send LIDAR free sector data to Arduino.

        Protocol format: s{angle}s{angle}s...#

        Example: "s0s5s10s15s340s345s350s355#"

        Args:
            free_sectors: List of obstacle-free sector angles (degrees)
        """
        if not free_sectors:
            return

        # Build message string
        message = "s"
        for sector in free_sectors:
            message += f"{int(sector)}s"
        message += "#"

        # Send to Arduino
        self.arduino.write(message.encode())

    def readline(self) -> str:
        """Read one line from Arduino.

        Returns:
            Decoded string from Arduino serial output
        """
        try:
            return self.arduino.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            return ""

    def read_available(self) -> List[str]:
        """Read all available lines from Arduino.

        Returns:
            List of all available lines
        """
        lines = []
        while self.arduino.in_waiting > 0:
            line = self.readline()
            if line:
                lines.append(line)
        return lines

    def clear_buffer(self):
        """Clear the input buffer."""
        self.arduino.reset_input_buffer()

    def close(self):
        """Close the serial connection."""
        if self.arduino.is_open:
            self.arduino.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures connection is closed."""
        self.close()
