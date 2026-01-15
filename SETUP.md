# Jetson Orin Nano + RealSense D435i + ROS 2 Humble Setup

This guide documents the specific steps required to get the Intel RealSense D435i working on an NVIDIA Jetson Orin Nano running JetPack 6 (Ubuntu 22.04) with ROS 2 Humble.

## 📋 Prerequisites
* **Hardware:** NVIDIA Jetson Orin Nano
* **Camera:** Intel RealSense D435i
* **OS:** JetPack 6 (Ubuntu 22.04)
* **Microcontroller:** ESP32 (for Micro-ROS section)

---

## 🛠️ Part 1: Install Low-Level Drivers (LibRealSense)
*Follow repository instructions to compile the kernel modules and SDK from source to ensure hardware acceleration works.*

    *(Ensure the camera opens and streams before proceeding).*

---

## 🐢 Part 2: Install ROS 2 Humble
*Standard Debian installation.*

1.  **Set Locale:**
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add Sources:**
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install Packages:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop ros-dev-tools
    ```
4.  **Configure Environment:**
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

---

## 📸 Part 3: Build RealSense ROS 2 Wrapper
*CRITICAL: We must build from source to link against the custom driver in `/usr/local`. Do not use `apt install ros-humble-realsense2-camera`.*

1.  **Create Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone the Compatible Version Directly:**
    *We specifically clone version 4.56.1 because it matches the Jetson's LibRealSense v2.56 driver. Newer versions (ros2-master) will fail to build.*
    ```bash
    git clone -b 4.56.1 [https://github.com/realsenseai/realsense-ros.git](https://github.com/realsenseai/realsense-ros.git)
    ```

3.  **Install Dependencies:**
    *We skip `librealsense2` keys to prevent `apt` from overwriting our custom driver.*
    ```bash
    cd ~/ros2_ws
    rosdep init
    rosdep update
    rosdep install -i --from-path src --rosdistro humble --skip-keys=librealsense2 -y
    ```
    *Manually install common missing dependencies:*
    ```bash
    sudo apt install ros-humble-diagnostic-updater ros-humble-cv-bridge -y
    ```

4.  **Build with Custom Flags:**
    *We must explicitly tell CMake where the JetsonHacks driver is located.*
    ```bash
    colcon build --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
    ```

5.  **Source the Workspace:**
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## 🚀 Part 4: Running the Camera & Visualization

1.  **Launch Camera (with Pointcloud):**
    ```bash
    ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
    ```
2.  **Visualize in RViz:**
    * Run `rviz2`
    * Set **Fixed Frame**: `camera_link`
    * **Add** -> **PointCloud2**
    * Set **Topic**: `/camera/depth/color/points`
    * Set **Style**: Points

---

## 📡 Part 5: Micro-ROS Setup (ESP32)

1.  **Create Workspace & Clone:**
    ```bash
    mkdir -p ~/microros_ws/src
    cd ~/microros_ws
    git clone -b humble [https://github.com/micro-ROS/micro_ros_setup.git](https://github.com/micro-ROS/micro_ros_setup.git) src/micro_ros_setup
    ```
2.  **Build Tools:**
    ```bash
    sudo apt update && rosdep update
    rosdep install --from-paths src --ignore-src -y
    colcon build
    source install/local_setup.bash
    ```
3.  **Build Agent:**
    ```bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash
    ```
4.  **Permissions:**
    ```bash
    sudo usermod -a -G dialout $USER
    # REBOOT REQUIRED HERE
    ```
5.  **Run Agent:**
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
    ```
