#!/bin/bash
# Setup Script for Raspberry Pi Robot Navigation Environment
# Run with: bash scripts/setup_environment.sh

set -e  # Exit on error

echo "=================================================="
echo "Robot Navigation System - Environment Setup"
echo "=================================================="
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ]; then
    echo "Warning: This doesn't appear to be a Raspberry Pi"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "Step 1/5: Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install system dependencies
echo "Step 2/5: Installing system dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    libcamera-dev \
    python3-libcamera \
    python3-picamera2 \
    i2c-tools \
    libatlas-base-dev \
    libopenblas-dev

# Install Python dependencies
echo "Step 3/5: Installing Python packages..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Setup camera
echo "Step 4/5: Configuring camera..."
if ! grep -q "^dtoverlay=vc4-kms-v3d" /boot/config.txt; then
    echo "dtoverlay=vc4-kms-v3d" | sudo tee -a /boot/config.txt
fi

# Configure I2C for IMU
echo "Step 5/5: Enabling I2C..."
if ! grep -q "^i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi
sudo modprobe i2c-dev

# Add user to dialout group for serial access
echo "Adding user to dialout group for serial/USB access..."
sudo usermod -a -G dialout $USER

# Setup LIDAR permissions
echo "Setting up LIDAR USB permissions..."
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
fi

echo ""
echo "=================================================="
echo "Setup Complete!"
echo "=================================================="
echo ""
echo "Next steps:"
echo "1. Reboot the Raspberry Pi: sudo reboot"
echo "2. After reboot, test camera: libcamera-hello"
echo "3. Check I2C devices: i2cdetect -y 1"
echo "4. Calibrate camera: cd calibration && python3 calibrate_camera.py"
echo "5. Update config files in config/ directory"
echo ""
echo "To run the system:"
echo "  Task 1: cd src/raspberry_pi && python3 task1_aruco_localization.py"
echo "  Task 2: cd src/raspberry_pi && python3 task2_slam_navigation.py"
echo ""
