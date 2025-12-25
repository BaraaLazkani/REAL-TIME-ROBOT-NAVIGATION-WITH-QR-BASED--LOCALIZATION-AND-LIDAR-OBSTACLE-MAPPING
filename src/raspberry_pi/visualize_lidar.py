#!/usr/bin/env python3
'''RPLidar polar plot visualization using threading (simplified & working version)'''

import threading
import queue
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# ============ Configuration ============
PORT = '/dev/ttyUSB0'
MAX_DIST = 1500
MIN_DIST = 1
SECTOR_WIDTH = 5
PLOT_RADIUS = 1000
SCAN_INTERVAL = 50  # ms
QUEUE_SIZE = 3

# ============ Shared Data ============
scan_queue = queue.Queue(maxsize=QUEUE_SIZE)

# ============ Lidar Reader Thread ============

def scan_loop():
    for scan in lidar.iter_scans():
        if scan_queue.full():
            scan_queue.get_nowait()
        scan_queue.put(scan)

# ============ Plot Updater ============
def update(frame, scatter):
    try:
        scan = scan_queue.get(timeout=1)
      
    except queue.Empty:
        return scatter,

    # Group into angle sectors
    sector_data = {a: [] for a in range(0, 360, SECTOR_WIDTH)}
    for _, angle, dist in scan:
        if MIN_DIST <= dist <= MAX_DIST:
            sector = (int(angle // SECTOR_WIDTH) * SECTOR_WIDTH) % 360
            sector_data[sector].append(dist)

    # Average each sector
    angles = []
    distances = []
    for sector, dists in sector_data.items():
        if len(dists)>2:
            print(sector)
            angles.append(np.radians(sector))
            distances.append(np.mean(dists))

    # Update scatter plot
    if angles:
        scatter.set_offsets(np.c_[angles, distances])
        scatter.set_array(np.array(distances))  # color by distance
    return scatter,

# ============ Main ============
lidar = RPLidar(PORT, baudrate=115200, timeout=3)

# Clean startup
try:
    lidar._serial_port.reset_input_buffer()
    lidar.stop()
    lidar.stop_motor()
except:
    pass

thread = threading.Thread(target=scan_loop, daemon=True)
thread.start()

# Setup polar plot
fig = plt.figure()
ax = plt.subplot(111, projection='polar')
scatter = ax.scatter([], [], s=5, c=[], cmap='Greys_r', lw=0)
ax.set_rmax(PLOT_RADIUS)
ax.grid(True)

# Start animation
ani = animation.FuncAnimation(fig, update, fargs=(scatter,), interval=SCAN_INTERVAL)

# Show plot and clean up after close
try:
    plt.show()
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Lidar safely stopped.")
