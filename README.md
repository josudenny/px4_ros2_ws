# PX4 SITL + ROS 2 Jazzy + Gazebo on WSL2

Student setup guide for:

- Windows 11
- WSL2
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- PX4 Autopilot
- Gazebo
- Micro XRCE-DDS Agent
- QGroundControl
- PX4 ROS 2 packages
- ROS 2 PX4 joystick teleoperation

---

# 1. Install Ubuntu 24.04 in WSL2

Run in **Windows PowerShell as Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Check WSL:

```powershell
wsl --status
wsl -l -v
```

After Ubuntu opens, create your Linux username and password.

---

# 2. Install ROS 2 Jazzy and Required Terminal Tools

Run inside Ubuntu:

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop terminator gnome-terminal

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check:

```bash
echo $ROS_DISTRO
ros2 --help
```

Expected:

```text
jazzy
```

---

# 3. Create the ROS 2 Workspace and Clone the Teleoperation Project

```bash
cd ~
mkdir -p px4_ros2_ws/src
cd px4_ros2_ws/src

git clone https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick.git --recursive
```

---

# 4. Install ROS 2 Workspace Dependencies

```bash
cd ~/px4_ros2_ws

sudo apt install -y python3-rosdep colcon

sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

---

# 5. Install PX4 Autopilot

```bash
cd ~

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

cd ~/PX4-Autopilot

make px4_sitl
```

---

# 6. Install Micro XRCE-DDS Agent

```bash
cd ~

git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

cd ~/Micro-XRCE-DDS-Agent
mkdir -p build
cd build

cmake ..
make

sudo make install
sudo ldconfig /usr/local/lib/
```

---

# 7. Add PX4 ROS 2 Packages

```bash
cd ~/px4_ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

cd ~/px4_ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build

source install/setup.bash
```

---

# 8. Verify ROS 2 ↔ PX4 Packages

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 topic list | grep fmu
```

PX4 topics should appear once PX4 and the Micro XRCE-DDS Agent are running.

---

# 9. Check the WSL Network Interface

Run:

```bash
ip addr | grep eth0
```

Find the IPv4 address assigned to `eth0`.

Example:

```text
inet 172.26.74.175/20
```

The IP may change after restarting WSL.

---

# 10. Start Micro XRCE-DDS Agent

Open a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Leave this terminal running.

---

# 11. Start PX4 SITL with Gazebo X500

Open another terminal:

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500
```

Leave this terminal running.

---

# 12. Start PX4 SITL in Baylands

For the Baylands environment:

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

Leave this terminal running.

---

# 13. Check PX4 MAVLink

Inside the PX4 shell:

```text
mavlink status
```

The GCS MAVLink instance should use:

```text
UDP 18570
remote port 14550
```

---

# 14. QGroundControl

Install and run **QGroundControl on Windows**.

For the WSL2 setup, PX4 uses its MAVLink GCS connection through the WSL network interface.

Check the current WSL IP whenever WSL is restarted:

```bash
ip addr | grep eth0
```

---

# 15. Build the PX4 Bringup Package

After the bringup package is available in the workspace:

```bash
cd ~/px4_ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --packages-select px4_bringup

source install/setup.bash
```

---

# 16. Launch PX4 Bringup

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 launch px4_bringup minimal.launch.py
```

---

# 17. Typical Multi-Terminal Workflow

For development, use separate terminals.

## Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

## Terminal 2 — Micro XRCE-DDS Agent

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

## Terminal 3 — ROS 2 Environment

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
```

## Terminal 4 — ROS 2 Monitoring

```bash
ros2 topic list
```

To specifically check PX4 topics:

```bash
ros2 topic list | grep fmu
```

---

# 18. Useful Daily Commands

## Start ROS 2 environment

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
```

## Start XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

## Start PX4 X500

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500
```

## Start PX4 X500 Baylands

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

## Check ROS 2 topics

```bash
ros2 topic list
```

## Check PX4 ROS 2 topics

```bash
ros2 topic list | grep fmu
```

## Check WSL IP

```bash
ip addr | grep eth0
```

---

# 19. Final Environment

```text
Windows 11
│
├── QGroundControl
│
└── WSL2
    │
    └── Ubuntu 24.04
        │
        ├── ROS 2 Jazzy
        ├── PX4 Autopilot
        ├── Gazebo
        ├── Micro XRCE-DDS Agent
        │
        └── ~/px4_ros2_ws
            ├── ROS2-PX4_Drone_Teleoperation_Using_Joystick
            ├── px4_msgs
            └── px4_ros_com
```

---

# Important

This guide uses **ROS 2 Jazzy** with **Ubuntu 24.04**.

Use:

```bash
source /opt/ros/jazzy/setup.bash
```

Do not use the old Humble command:

```bash
source /opt/ros/humble/setup.bash
```

For a fresh student installation, keep the environment consistently on:

```text
Ubuntu 24.04
ROS 2 Jazzy
PX4
Gazebo
Micro XRCE-DDS Agent
```
