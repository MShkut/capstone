# Librealsense Kernel Module Installation

**Last Updated:** March 17, 2025

This script installs and loads custom-compiled kernel modules to support Intel® RealSense™ cameras (via Librealsense) on systems running kernel version 5.15.148-tegra. These modules provide enhanced support for USB video devices and HID sensors, including accelerometers and gyroscopes.

## Modules

1. **`uvcvideo.ko`**
   - **Description:** USB Video Class driver module for webcam/video device support, modified for Librealsense compatibility.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/media/usb/uvc/uvcvideo.ko`

2. **`hid-sensor-accel-3d.ko`**
   - **Description:** HID sensor driver for 3D accelerometer support, used by RealSense devices.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/iio/accel/hid-sensor-accel-3d.ko`

3. **`hid-sensor-iio-common.ko`**
   - **Description:** Common Industrial I/O framework components for HID sensors, supporting RealSense functionality.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko`

4. **`hid-sensor-hub.ko`**
   - **Description:** HID Sensor Hub driver for managing multiple HID sensors in RealSense devices.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/hid/hid-sensor-hub.ko`

5. **`hid-sensor-trigger.ko`**
   - **Description:** Trigger support for HID sensors in the IIO framework, required for RealSense sensor data.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko`

6. **`hid-sensor-gyro-3d.ko`**
   - **Description:** HID sensor driver for 3D gyroscope support, used by RealSense devices.
   - **Installed to:** `/lib/modules/5.15.148-tegra/kernel/drivers/iio/gyro/hid-sensor-gyro-3d.ko`

## Requirements

- Must be run with sudo privileges.
- All `.ko` files must be present in the current directory.
- Target system must be running kernel version 5.15.148-tegra.

## Usage Instructions

1. **Prepare the Environment:**
   - Ensure all listed `.ko` files are in the current working directory.
   - Verify you're running kernel version `5.15.148-tegra` (check with `uname -r`).

2. **Run the Script:**
   ```bash
   sudo ./install-librealsense-modules.sh
   ```
   - The script will:
     - Verify root privileges
     - Check for all required files
     - Create necessary directories
     - Copy the modules to their destinations
     - Update module dependencies with `depmod`
     - Unload any existing versions of these modules (if loaded) and load the new ones with `modprobe`
     - Provide feedback on success or failure

3. **Verify Functionality:**
   - Check loaded modules: `lsmod | grep -E 'uvcvideo|hid_sensor'`
   - Test with Librealsense tools (e.g., `rs-enumerate-devices`).

## Notes

- These modules are specifically compiled for Librealsense support and may differ from stock kernel modules.
- The script unloads existing versions of these modules before loading the new ones to ensure the updated versions are used.
- Backup existing modules in the target directories if you need to revert changes.
- If issues persist after running the script, a reboot may be required to fully initialize the modules.
- The tar file was created as follows:
 tar -czf install-modules.tar.gz install-modules
 sha256sum install-modules.tar.gz > install-modules.tar.gz.sha256
