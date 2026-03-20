#!/bin/bash

# Librealsense Kernel Module Installation Script

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)"
    exit 1
fi

# Base directory for kernel modules
BASE_DIR="/lib/modules/5.15.148-tegra/kernel"

# List of files to check and their corresponding module names
FILES=(
    "uvcvideo.ko:uvcvideo"
    "hid-sensor-accel-3d.ko:hid_sensor_accel_3d"
    "hid-sensor-iio-common.ko:hid_sensor_iio_common"
    "hid-sensor-hub.ko:hid_sensor_hub"
    "hid-sensor-trigger.ko:hid_sensor_trigger"
    "hid-sensor-gyro-3d.ko:hid_sensor_gyro_3d"
)

# Check if all files exist in current directory first
echo "Checking for required files..."
for entry in "${FILES[@]}"; do
    file="${entry%%:*}"  # Extract filename before colon
    if [ ! -f "$file" ]; then
        echo "Error: $file not found in current directory"
        exit 1
    fi
done
echo "All required files found."

# Create necessary directory paths if they don't exist
mkdir -p "$BASE_DIR/drivers/media/usb/uvc"
mkdir -p "$BASE_DIR/drivers/iio/accel"
mkdir -p "$BASE_DIR/drivers/iio/common/hid-sensors"
mkdir -p "$BASE_DIR/drivers/hid"
mkdir -p "$BASE_DIR/drivers/iio/gyro"

# Copy kernel modules with error checking
echo "Installing kernel modules..."
sudo cp uvcvideo.ko "$BASE_DIR/drivers/media/usb/uvc/uvcvideo.ko" || { echo "Failed to install uvcvideo.ko"; exit 1; }
sudo cp hid-sensor-accel-3d.ko "$BASE_DIR/drivers/iio/accel/hid-sensor-accel-3d.ko" || { echo "Failed to install hid-sensor-accel-3d.ko"; exit 1; }
sudo cp hid-sensor-iio-common.ko "$BASE_DIR/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko" || { echo "Failed to install hid-sensor-iio-common.ko"; exit 1; }
sudo cp hid-sensor-hub.ko "$BASE_DIR/drivers/hid/hid-sensor-hub.ko" || { echo "Failed to install hid-sensor-hub.ko"; exit 1; }
sudo cp hid-sensor-trigger.ko "$BASE_DIR/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko" || { echo "Failed to install hid-sensor-trigger.ko"; exit 1; }
sudo cp hid-sensor-gyro-3d.ko "$BASE_DIR/drivers/iio/gyro/hid-sensor-gyro-3d.ko" || { echo "Failed to install hid-sensor-gyro-3d.ko"; exit 1; }

# Update module dependencies
echo "Updating module dependencies..."
depmod -a 5.15.148-tegra || { echo "Failed to update module dependencies"; exit 1; }

# Load the modules
echo "Loading kernel modules..."
for entry in "${FILES[@]}"; do
    module="${entry#*:}"  # Extract module name after colon
    # Unload if already loaded, ignore errors if not loaded
    modprobe -r "$module" 2>/dev/null
    # Load the module
    modprobe "$module" || { echo "Failed to load $module"; exit 1; }
done

echo "All kernel modules installed and loaded successfully"
