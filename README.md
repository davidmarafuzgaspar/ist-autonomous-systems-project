# AlphaBot2 + ROS 2 (Docker + Foxglove) Setup

---

# Part 1 — Installation

## 1. Install Docker (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install docker.io -y
sudo systemctl enable --now docker
```

```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## 2. Pull ROS 2 Humble Docker Image

```bash
docker pull osrf/ros:humble-desktop-full
```

This image includes:

* ROS 2 Humble
* RViz2
* RQT tools

---

## 3. Create Docker Launch Script

```bash
nano ~/ros2_humble.sh
```

Paste:

```bash
#!/bin/bash

xhost +local:docker

docker run -it --rm \
  --network host \
  --env ROS_DOMAIN_ID=50 \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name ros2_humble \
  osrf/ros:humble-desktop-full \
  bash -c "source /opt/ros/humble/setup.bash && bash"
```

Make it executable:

```bash
chmod +x ~/ros2_humble.sh
```

---

## 4. Install Foxglove Studio (GUI)

Download and install Foxglove Studio from the official website:


```bash
sudo snap install foxglove-studio
```

---

## 5. Install Foxglove Bridge (inside Docker)

Start your container:

```bash
~/ros2_humble.sh
```

Inside the container:

```bash
apt update
apt install ros-humble-foxglove-bridge
```

---

# Part 2 — System Usage

---

## 1. Start the Robot

### Terminal 1 — Main drivers

```bash
ssh deec@10.16.140.68
```

```bash
ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — Motion driver

```bash
ssh deec@10.16.140.68
```

```bash
ros2 run alphabot2 motion_driver
```

---

## 2. Start Docker ROS Environment

### Terminal 3

```bash
~/ros2_humble.sh
```

---

## 3. Load Robot Workspace

If using custom messages (e.g. obstacles) inside docker:

```bash
cd ~/ros2_ws
source install/setup.bash
```

---

## 4. Start Foxglove Bridge

Inside Docker:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

You should see:

```text
Server listening on port 8765
```

---

## 5. Connect Foxglove Studio

Open Foxglove Studio:

* Click **Open Connection**
* Select **WebSocket**
* Enter:

```text
ws://localhost:8765
```

---

## 6. Verify ROS Connection

Inside Docker:

```bash
ros2 topic list
```

You should see topics like:

* `/alphabot2/cmd_vel`
* `/alphabot2/image_raw`
* `/tf`
* `/alphabot2/obstacles`

---

# Part 3 — Control & Visualization

---

## Publish velocity commands

```bash
ros2 topic pub --rate 1 /alphabot2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

---

## Keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Camera visualization (RQT)

```bash
ros2 run rqt_image_view rqt_image_view
```

---
