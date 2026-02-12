# Robot Launch Guide

## Prerequisites
- ESP32 flashed with modified firmware (IMU subscription enabled)
- Jetson Orin Nano with all workspaces built
- RealSense D435i connected
- ESP32 connected via USB

---

## 🚀 Quick Start

### 1. Build and Flash ESP32 Firmware

```bash
cd ~/linorobot2_hardware_hippo_esp32_fix_ws/firmware
pio run
pio run --target upload --upload-port /dev/ttyUSB0
```

---

## 🤖 Launch Sequence

### Terminal 1: Micro-ROS Agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected output:** `agent connected` messages

---

### Terminal 2: RealSense Camera
```bash
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

**Expected output:** Camera streaming messages

---

### Terminal 3: ekf
```bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/src/config/ekf_config.yaml
```

### Terminal 4: Teleoperation
```bash
# Install teleop (first time only)
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Forward
- `,` - Backward
- `j` - Rotate left
- `l` - Rotate right
- `k` - Stop
- `q/z` - Increase/decrease max speeds

---

## ⚠️ Current Limitations

**PID is disabled** - Motors run at full PWM (lines 428-431 in firmware.ino)
- Robot only has ON/OFF control
- Cannot control speed accurately
- Will run at maximum speed when commanded

**To fix:** Calibrate robot dimensions and re-enable PID control

---

## 🔧 Troubleshooting

### ESP32 not connecting
```bash
# Check device
ls /dev/ttyUSB*

# Check permissions
sudo usermod -a -G dialout $USER
# Then reboot
```

### Topics not appearing
```bash
# Check agent connection
ros2 topic list

# Restart agent (Terminal 1)
# Ctrl+C, then re-run command
```

### Camera not working
```bash
# Test camera separately
realsense-viewer

# Check USB connection
rs-enumerate-devices
```

---

## 📋 Next Steps (Robot Calibration)

1. **Measure wheel diameter** → Update `WHEEL_DIAMETER` in config
2. **Measure track width** → Update `TRACK_WIDTH` in config
3. **Count encoder ticks** → Update `COUNTS_PER_REV1/2` in config
4. **Test motor directions** → Adjust `MOTOR*_INV` flags
5. **Measure max RPM** → Update `MOTOR_MAX_RPM` in config
6. **Re-enable PID** → Uncomment lines 428-429, comment 430-431
7. **Tune PID gains** → Adjust `K_P`, `K_I`, `K_D` values
