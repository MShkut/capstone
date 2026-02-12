#!/bin/bash

# Robot Launch Script
# Automates the startup sequence for the ESP32-based robot with RealSense camera

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   Robot Launch Sequence Starting${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if required devices exist
echo -e "${YELLOW}Checking prerequisites...${NC}"

if [ ! -e "/dev/ttyUSB0" ]; then
    echo -e "${RED}ERROR: ESP32 not found at /dev/ttyUSB0${NC}"
    echo "Please connect the ESP32 and try again."
    exit 1
fi

echo -e "${GREEN}✓ ESP32 found at /dev/ttyUSB0${NC}"
echo ""

# Function to launch in new terminal
launch_terminal() {
    local title=$1
    local command=$2
    
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$command; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -T "$title" -e bash -c "$command; exec bash" &
    else
        echo -e "${RED}ERROR: No suitable terminal emulator found${NC}"
        echo "Please install gnome-terminal or xterm"
        exit 1
    fi
    
    sleep 2  # Give terminal time to start
}

# Launch Terminal 1: Micro-ROS Agent
echo -e "${BLUE}[1/4] Launching Micro-ROS Agent...${NC}"
launch_terminal "Micro-ROS Agent" \
    "source ~/microros_ws/install/setup.bash && \
     echo -e '${GREEN}Starting Micro-ROS Agent...${NC}' && \
     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"

echo -e "${GREEN}✓ Micro-ROS Agent terminal launched${NC}"
sleep 3

# Launch Terminal 2: RealSense Camera
echo -e "${BLUE}[2/4] Launching RealSense Camera...${NC}"
launch_terminal "RealSense Camera" \
    "echo -e '${GREEN}Starting RealSense Camera...${NC}' && \
     ros2 launch realsense2_camera rs_launch.py \
     enable_gyro:=true \
     enable_accel:=true \
     unite_imu_method:=2"

echo -e "${GREEN}✓ RealSense Camera terminal launched${NC}"
sleep 5

# Launch Terminal 3: EKF Node
echo -e "${BLUE}[3/4] Launching EKF Node...${NC}"
launch_terminal "EKF Node" \
    "echo -e '${GREEN}Starting EKF Node...${NC}' && \
     ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/src/config/ekf_config.yaml"

echo -e "${GREEN}✓ EKF Node terminal launched${NC}"
sleep 3

# Launch Terminal 4: Topic Monitor
echo -e "${BLUE}[4/4] Launching Topic Monitor...${NC}"
launch_terminal "ROS2 Topics" \
    "echo -e '${YELLOW}Waiting for topics to initialize...${NC}' && \
     sleep 3 && \
     echo -e '${GREEN}Available ROS2 Topics:${NC}' && \
     ros2 topic list && \
     echo -e '' && \
     echo -e '${YELLOW}Press Enter to refresh topic list${NC}' && \
     read && \
     ros2 topic list"

echo -e "${GREEN}✓ Topic Monitor terminal launched${NC}"
echo ""

# Launch Terminal 5: Teleoperation
echo -e "${BLUE}[5/5] Launching Teleoperation...${NC}"
launch_terminal "Teleoperation" \
    "sleep 12 && \
     echo -e '${GREEN}Starting Teleoperation...${NC}' && \
     ros2 run teleop_twist_keyboard teleop_twist_keyboard"

echo -e "${GREEN}✓ Teleoperation terminal launched${NC}"
echo ""

# Wait for everything to stabilize
echo -e "${YELLOW}Waiting for all systems to initialize...${NC}"
sleep 3

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Robot Launch Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${RED}⚠️  WARNING: PID is currently disabled!${NC}"
echo -e "${YELLOW}   Motors run at full PWM (ON/OFF only)${NC}"
echo ""
echo -e "${BLUE}5 terminals launched:${NC}"
echo "  1. Micro-ROS Agent"
echo "  2. RealSense Camera"
echo "  3. EKF Node"
echo "  4. Topic Monitor"
echo "  5. Teleoperation (ready to control)"
echo ""
echo -e "${YELLOW}Expected topics:${NC}"
echo "  • /cmd_vel"
echo "  • /odom/unfiltered"
echo "  • /odom/filtered (from EKF)"
echo "  • /imu/data (from ESP32)"
echo "  • /camera/imu (from RealSense)"
echo "  • /camera/depth/image_rect_raw"
echo "  • /camera/color/image_raw"
echo ""