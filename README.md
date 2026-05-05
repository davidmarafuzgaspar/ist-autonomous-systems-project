
# AlphaBot2 + ROS 2 (Docker + Foxglove) Setup — Improved Version

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

## 2. Pull ROS 2 Humble Base Image

```bash
docker pull osrf/ros:humble-desktop-full
```

Includes:

* ROS 2 Humble
* RViz2
* RQT tools

---

## 3. (IMPORTANT) Create a Persistent ROS + AlphaBot Image

Instead of installing tools every time, we bake them into an image.

```bash
mkdir ~/alphabot_docker
cd ~/alphabot_docker
nano Dockerfile
```

### 📦 Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop-full

# Install useful ROS tools
RUN apt update && apt install -y \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
```

---

## 4. Build the Image (ONE TIME)

```bash
docker build -t alphabot2_ros2 .
```

---

## 5. Create One-Command Launcher

```bash
nano ~/run_alphabot.sh
```

### Run Script

```bash
#!/bin/bash

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}

xhost +local:docker

docker run -it --rm \
  --net=host \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/alphabot2_ws_src:/ros2_ws \
  alphabot2_ros2
```

Make executable:

```bash
chmod +x ~/run_alphabot.sh
```

---

## ✅ Now you NEVER reinstall anything again

Change domain (if needed) and run:


```bash
ROS_DOMAIN_ID=12 ~/run_alphabot.sh
```

---

## 6. Install Foxglove Studio (Host Machine)

```bash
sudo snap install foxglove-studio
```

---

# Part 2 — System Usage

---

## 1. Start Robot (SSH)

### Terminal 1 — Main drivers

```bash
ssh deec@10.16.140.68
ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — Motion driver

```bash
ssh deec@10.16.140.68
ros2 run alphabot2 motion_driver
```

---

## 2. Start ROS Docker Environment

### Terminal 3

```bash
~/run_alphabot.sh
```

---

## 3. Load Workspace (inside Docker)

```bash
cd /ros2_ws
source install/setup.bash
```

---

## 4. Start Foxglove Bridge (inside Docker)

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

* Open Connection
* WebSocket
* Enter:

```text
ws://localhost:8765
```

---

## 6. Verify ROS Topics

```bash
ros2 topic list
```

Expected:

* `/alphabot2/cmd_vel`
* `/alphabot2/image_raw`
* `/tf`
* `/alphabot2/obstacles`

---

# Part 3 — Control & Debug Tools

---

## Manual velocity control

```bash
ros2 topic pub --rate 1 /alphabot2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

---

## Keyboard control (teleop)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Camera viewer (RQT)

```bash
ros2 run rqt_image_view rqt_image_view
```