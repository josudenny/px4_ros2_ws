# PX4 SITL + ROS 2 Jazzy + Gazebo on WSL2
## Setup, Troubleshooting, Fixes, and Useful Commands

**Environment**

- Windows 11
- WSL2
- Ubuntu 24.04 LTS
- ROS 2 Jazzy Desktop
- PX4 Autopilot
- Gazebo Sim
- QGroundControl on Windows
- Micro XRCE-DDS Agent
- ROS 2 workspace: `~/px4_ros2_ws`

---

# 1. WSL2 Installation

Run these commands from **Windows PowerShell as Administrator**.

```powershell
wsl --status
wsl -l -v
wsl --install -d Ubuntu-24.04
```

Useful WSL commands:

```powershell
wsl --shutdown
wsl -l -v
```

### Installation issue encountered

The Ubuntu download stopped around 64.5% after an internet connection interruption.

Recovery:

1. Press `Ctrl+C` to cancel the stuck download.
2. Check installed distributions:

```powershell
wsl -l -v
```

3. Shut down WSL if necessary:

```powershell
wsl --shutdown
```

4. Retry:

```powershell
wsl --install -d Ubuntu-24.04
```

---

# 2. Ubuntu Basic Setup

Inside Ubuntu 24.04:

```bash
sudo apt update
sudo apt upgrade -y
```

---

# 3. ROS 2 Jazzy Installation

Ubuntu 24.04 was paired with ROS 2 Jazzy.

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Install ROS 2 Jazzy Desktop:

```bash
sudo apt install -y ros-jazzy-desktop
```

Source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Make it permanent:

```bash
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

# 4. ROS 2 Verification

### Terminal 1

```bash
ros2 run demo_nodes_cpp talker
```

### Terminal 2

```bash
ros2 run demo_nodes_py listener
```

Expected listener output:

```text
I heard: [Hello World: 1]
I heard: [Hello World: 2]
I heard: [Hello World: 3]
```

---

# 5. Multiple Terminal Setup

For PX4 work, multiple terminals are useful.

Install Terminator:

```bash
sudo apt update
sudo apt install -y terminator
```

Launch:

```bash
terminator
```

The ROS 2 launch file used in this setup specifically calls `gnome-terminal`, so install it too:

```bash
sudo apt update
sudo apt install -y gnome-terminal
```

Test:

```bash
gnome-terminal
```

**Note:** Terminator does not replace `gnome-terminal` for a launch file that explicitly executes `gnome-terminal`.

---

# 6. PX4 Directory

The PX4 source directory used in this setup is:

```text
~/PX4-Autopilot
```

Go to it:

```bash
cd ~/PX4-Autopilot
```

---

# 7. Find the WSL2 Network Interface

Check the WSL network interface:

```bash
ip addr | grep eth0
```

The IP found during setup was:

```text
eth0
172.26.74.175
```

**Important:** The WSL IP can change when WSL restarts. Check it again when troubleshooting Windows ↔ WSL networking.

---

# 8. PX4 SITL + Gazebo

Normal X500:

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500
```

Baylands:

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

`PX4_NET_INTERFACE=eth0` was used so PX4 networking uses the WSL `eth0` interface.

---

# 9. QGroundControl Connection Problem

QGroundControl was installed on **Windows**, while PX4 SITL was running inside **WSL2**.

PX4 initially printed:

```text
MAVLink only on localhost
```

and:

```text
Broadcast enabled: NO
```

PX4 also reported:

```text
Preflight Fail: No connection to the GCS
```

This meant the simulation itself was running, but Windows QGroundControl could not receive PX4 MAVLink traffic.

---

# 10. PX4 MAVLink Configuration Investigation

Inside the PX4 shell:

```text
mavlink status
```

The GCS MAVLink instance showed:

```text
instance #0
transport protocol: UDP
local port: 18570
remote port: 14550
Broadcast enabled: NO
```

So the relevant ports were:

```text
PX4 local UDP port: 18570
QGroundControl remote port: 14550
```

---

# 11. MAV_0_BROADCAST Test

We tested:

```text
param set MAV_0_BROADCAST 1
```

Then:

```text
param show MAV_0_BROADCAST
```

It temporarily showed:

```text
MAV_0_BROADCAST = 1
```

However, after restarting SITL, the effective MAVLink configuration returned to:

```text
Broadcast enabled: NO
```

Therefore this parameter change was not used as the final persistent solution for the current SITL configuration.

---

# 12. Final WSL PX4 Network Configuration

The approach used for PX4 networking was:

```bash
export PX4_NET_INTERFACE=eth0
```

Then launch PX4:

```bash
make px4_sitl gz_x500
```

or:

```bash
make px4_sitl gz_x500_baylands
```

### Network concept

```text
Windows
│
└── QGroundControl
        │
        │ MAVLink / UDP
        ↓
     WSL2 eth0
   172.26.74.175
        │
        ↓
    PX4 SITL
        │
        ↓
      Gazebo
```

---

# 13. MAVLink Router Attempt

We attempted:

```bash
sudo apt update
sudo apt install -y mavlink-router
```

Ubuntu 24.04 returned:

```text
E: Unable to locate package mavlink-router
```

Therefore MAVLink Router was **not** used in the final setup.

---

# 14. Micro XRCE-DDS Agent

PX4 starts its uXRCE-DDS client using:

```text
UDP agent IP: 127.0.0.1
port: 8888
```

Launch the agent manually with:

```bash
MicroXRCEAgent udp4 -p 8888
```

This provides the DDS bridge used for ROS 2 ↔ PX4 communication.

---

# 15. ROS 2 Workspace

Workspace:

```text
~/px4_ros2_ws
```

Build:

```bash
cd ~/px4_ros2_ws
colcon build
```

Source:

```bash
source ~/px4_ros2_ws/install/setup.bash
```

---

# 16. ROS 2 PX4 Bringup Package

Launch command:

```bash
ros2 launch px4_bringup minimal.launch.py
```

The launch file is intended to start:

1. PX4 SITL + Gazebo
2. Micro XRCE-DDS Agent

---

# 17. `gnome-terminal` Launch Error

The original ROS 2 launch failed with:

```text
FileNotFoundError: [Errno 2] No such file or directory: 'gnome-terminal'
```

This happened for both:

```text
px4_sitl
microxrce_agent
```

### Cause

The Python launch file explicitly used:

```python
'gnome-terminal'
```

but it was not installed.

### Fix

```bash
sudo apt update
sudo apt install -y gnome-terminal
```

---

# 18. Minimal Launch File for Baylands

The intended launch file is:

```python
#!/usr/bin/env python3

"""
PX4 Minimal Launch File
Launches:
1. PX4 SITL with Gazebo X500 in Baylands world
2. MicroXRCE-DDS Agent
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():

    px4_dir = os.path.expanduser('~/PX4-Autopilot')

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL - Baylands',
                '--',
                'bash',
                '-c',
                f'cd {px4_dir} && '
                f'export PX4_NET_INTERFACE=eth0 && '
                f'make px4_sitl gz_x500_baylands; '
                f'exec bash'
            ],
            output='screen',
            name='px4_sitl'
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE-DDS Agent',
                        '--',
                        'bash',
                        '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),
    ])
```

---

# 19. Rebuild the Bringup Package

After changing `minimal.launch.py`:

```bash
cd ~/px4_ros2_ws
colcon build --packages-select px4_bringup
source install/setup.bash
```

Launch:

```bash
ros2 launch px4_bringup minimal.launch.py
```

---

# 20. Useful Verification Commands

### Ubuntu version

```bash
lsb_release -a
```

### WSL kernel

```bash
uname -r
```

### ROS version

```bash
echo $ROS_DISTRO
ros2 --help
```

### WSL IP

```bash
ip addr | grep eth0
```

### PX4 MAVLink

Inside `pxh>`:

```text
mavlink status
```

### PX4 parameter

Inside `pxh>`:

```text
param show MAV_0_BROADCAST
```

### ROS 2 topics

```bash
ros2 topic list
```

### PX4 ROS 2 topics

```bash
ros2 topic list | grep fmu
```

---

# 21. Typical Multi-Terminal Workflow

### Terminal 1 — PX4

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

### Terminal 2 — Micro XRCE-DDS

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — ROS 2 workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
```

### Terminal 4 — ROS 2 monitoring

```bash
ros2 topic list
```

---

# 22. One-Command Reference

### Start X500 Baylands

```bash
cd ~/PX4-Autopilot
export PX4_NET_INTERFACE=eth0
make px4_sitl gz_x500_baylands
```

### Start XRCE Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Build ROS 2 workspace

```bash
cd ~/px4_ros2_ws
colcon build
source install/setup.bash
```

### Launch bringup

```bash
ros2 launch px4_bringup minimal.launch.py
```

### Check WSL network

```bash
ip addr | grep eth0
```

### Check PX4 MAVLink

```text
mavlink status
```

---

# 23. Troubleshooting Summary

| Problem | Cause | Fix |
|---|---|---|
| WSL had no distro | No Linux distribution installed | Install Ubuntu 24.04 |
| Ubuntu download stalled | Internet interruption | Cancel and retry installation |
| ROS distribution choice | Ubuntu 24.04 | Use ROS 2 Jazzy |
| Multiple terminal difficulty | Terminal/tab behavior | Windows Terminal / Terminator / separate windows |
| QGC not connecting | PX4 MAVLink localhost-only | Use WSL `eth0` networking |
| `No connection to the GCS` | Windows QGC could not receive MAVLink | Set `PX4_NET_INTERFACE=eth0` |
| `Broadcast enabled: NO` | SITL MAVLink restricted | Broadcast parameter was tested; network-interface approach used |
| `MAV_0_BROADCAST` did not persist | Parameter not retained in this SITL configuration | Use `PX4_NET_INTERFACE=eth0` |
| `mavlink-router` not found | Not available through configured Ubuntu APT repositories | Do not use it |
| `gnome-terminal` missing | Launch file explicitly invokes it | Install `gnome-terminal` |
| Default X500 world | Launch file used `gz_x500` | Use Baylands target |
| ROS launch failed to start processes | Missing terminal executable | Install `gnome-terminal` |

---

# 24. Final Architecture

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
        │
        ├── PX4 Autopilot
        │      └── PX4 SITL
        │
        ├── Gazebo Sim
        │      └── X500 / Baylands
        │
        ├── Micro XRCE-DDS Agent
        │      └── UDP 8888
        │
        └── ~/px4_ros2_ws
               └── px4_bringup
```

---

# 25. Important Notes

- The WSL IP address can change after restarting WSL.
- `PX4_NET_INTERFACE=eth0` is an environment variable for the current shell unless added to shell configuration.
- `MAV_0_BROADCAST` was tested but was not persistent in the current SITL parameter setup.
- `mavlink-router` was not part of the final solution.
- QGroundControl is running on Windows while PX4/ROS 2/Gazebo are running inside WSL2.
- The ROS 2 launch file uses `gnome-terminal`, so `gnome-terminal` must remain installed.
- The launch target `gz_x500_baylands` should be confirmed against the PX4 checkout if a future PX4 version changes available targets.
