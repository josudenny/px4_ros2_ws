# PX4 SITL + ROS 2 Jazzy + Gazebo — Student Setup Commands

## 1. Install Ubuntu 24.04 in WSL2

Run in **Windows PowerShell as Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Check WSL:

```powershell
wsl --status
wsl -l -v
```

---

## 2. Update Ubuntu

Inside Ubuntu:

```bash
sudo apt update
sudo apt upgrade -y
```

---

## 3. Install ROS 2 Jazzy

Install required tools:

```bash
sudo apt install -y software-properties-common curl
```

Enable Universe:

```bash
sudo add-apt-repository universe
```

Add the ROS repository key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package lists:

```bash
sudo apt update
```

Install ROS 2 Jazzy Desktop:

```bash
sudo apt install -y ros-jazzy-desktop
```

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Make it permanent:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check ROS 2:

```bash
echo $ROS_DISTRO
ros2 --help
```

---

## 4. Install Terminal Applications

Install Terminator:

```bash
sudo apt install -y terminator
```

Install GNOME Terminal:

```bash
sudo apt install -y gnome-terminal
```

---

## 5. Create the ROS 2 Workspace

```bash
cd ~
mkdir -p px4_ros2_ws/src
cd px4_ros2_ws/src
```

---

## 6. Clone the Teleoperation Project

```bash
git clone https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick.git --recursive
```

---

## 7. Install ROS 2 Workspace Dependencies

```bash
cd ~/px4_ros2_ws
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Install colcon:

```bash
sudo apt install colcon -y
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

## 8. Install PX4 Autopilot

Clone PX4:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run the PX4 setup script:

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Enter the PX4 directory:

```bash
cd ~/PX4-Autopilot
```

Build PX4 SITL:

```bash
make px4_sitl
```

---

## 9. Install Micro XRCE-DDS Agent

Clone the agent:

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```

Build:

```bash
cd ~/Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
```

Install:

```bash
sudo make install
sudo ldconfig /usr/local/lib/
```

Run the agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

---

## 10. Clone PX4 ROS 2 Packages

```bash
cd ~/px4_ros2_ws/src
```

Clone `px4_msgs`:

```bash
git clone https://github.com/PX4/px4_msgs.git
```

Clone `px4_ros_com`:

```bash
git clone https://github.com/PX4/px4_ros_com.git
```

---

## 11. Build the ROS 2 Workspace Again

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 12. Check PX4 ROS 2 Topics

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic list | grep fmu
```

---

# PX4 SITL + Gazebo

## 13. Find the WSL Network Interface

```bash
ip addr | grep eth0
```

The address shown for `eth0` is the WSL IP.

---

## 14. Start PX4 X500 SITL

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500
```

---

## 15. Start PX4 X500 in Baylands

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

---

# ROS 2 Bringup

## 16. Build the Bringup Package

```bash
cd ~/px4_ros2_ws
colcon build --packages-select px4_bringup
source install/setup.bash
```

---

## 17. Launch PX4 Bringup

```bash
ros2 launch px4_bringup minimal.launch.py
```

---

# Useful Commands During Development

## ROS 2

```bash
ros2 topic list
```

```bash
ros2 topic list | grep fmu
```

```bash
echo $ROS_DISTRO
```

## PX4

Inside the PX4 shell:

```text
mavlink status
```

## Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

---

# Typical Multi-Terminal Workflow

### Terminal 1 — PX4

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

### Terminal 2 — Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — ROS 2

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
```

### Terminal 4 — ROS 2 Monitoring

```bash
ros2 topic list
```

---

# Environment Summary

```text
Windows 11
└── WSL2
    └── Ubuntu 24.04
        ├── ROS 2 Jazzy
        ├── PX4 Autopilot
        ├── Gazebo
        ├── Micro XRCE-DDS Agent
        └── ~/px4_ros2_ws
            ├── ROS2-PX4_Drone_Teleoperation_Using_Joystick
            ├── px4_msgs
            └── px4_ros_com
```

**Important:** This setup uses **ROS 2 Jazzy**, so use:

```bash
source /opt/ros/jazzy/setup.bash
```

and not the older Humble command:

```bash
source /opt/ros/humble/setup.bash
```
