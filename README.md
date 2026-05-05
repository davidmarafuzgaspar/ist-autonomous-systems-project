# AlphaBot2 + ROS 2 (Docker + Foxglove) Setup

---

# 1. Setup (One-time installation)

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

## 2. Pull base ROS 2 image

```bash
docker pull osrf/ros:humble-desktop-full
```

---

## 3. Create persistent ROS + AlphaBot image

```bash
mkdir ~/alphabot_docker
cd ~/alphabot_docker
nano Dockerfile
```

### Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /alphabot2_ws
```

---

## 4. Build Docker image (one time only)

```bash
docker build -t alphabot2_ros2 .
```

---

## 5. Create runtime launcher script

```bash
nano ~/run_alphabot.sh
```

### Script

```bash
#!/bin/bash

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}

xhost +local:docker

docker run -it --rm \
  --net=host \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/alphabot2_ws:/alphabot2_ws \
  alphabot2_ros2
```

Make executable:

```bash
chmod +x ~/run_alphabot.sh
```

---

## 6. Install Foxglove Studio (host machine)

```bash
sudo snap install foxglove-studio
```

---

# 2. Workspace Setup (ROS 2 Development Workspace)

---

## 2.1 Create the workspace directory

On your PC, create the main workspace folder:

```bash id="ws1"
mkdir -p ~/alphabot2_ws/src
```

This creates:

```text id="ws2"
~/alphabot2_ws/
 └── src/
```

## 2.3 Add your robot source code

You now need to place your AlphaBot2 software into `src/`.

If you already have it on your robot, copy it once to your PC:

```bash id="ws4"
scp -r deec@10.16.140.68:~/alphabot2_ws/src ~/alphabot2_ws/
```

After this, your structure becomes:

```text id="ws5"
~/alphabot2_ws/src
 ├── alphabot2-ros2
 ├── image_common
 ├── image_transport_plugins
 ├── ros2_v4l2_camera
 ├── vision_opencv
```

These are ROS packages that provide:

* robot control (alphabot2-ros2)
* camera drivers (v4l2_camera)
* image processing (opencv, image_transport)

---

# 3. Usage (Every session workflow)

---

## 1. Start robot system (SSH)

### Terminal 1 — main robot drivers

```bash
ssh deec@10.16.140.68
ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — motion driver

```bash
ssh deec@10.16.140.68
ros2 run alphabot2 motion_driver
```

---

## 2. Start ROS Docker environment (local machine)

### Terminal 3

```bash
ROS_DOMAIN_ID=12 ~/run_alphabot.sh
```

---

## 3. Build workspace (only if code changed)

Inside Docker:

```bash
cd /alphabot2_ws
colcon build
```

Then source:

```bash
source install/setup.bash
```

---

## 4. Start Foxglove bridge (inside Docker)

```bash
ros2 run foxglove_bridge foxglove_bridge
```

Expected:

```
Server listening on port 8765
```

---

## 5. Connect Foxglove Studio (host machine)

* Open Foxglove Studio
* WebSocket connection
* Enter:

```
ws://localhost:8765
```

---

## 6. Verify ROS communication

Inside Docker:

```bash
ros2 topic list
```

Expected topics:

* `/alphabot2/cmd_vel`
* `/alphabot2/image_raw`
* `/tf`
* `/alphabot2/obstacles`

---

# 4. Examples of usage

---

## 1. Manual robot movement

```bash
ros2 topic pub --rate 1 /alphabot2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

---

## 2. Keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 3. Camera visualization

```bash
ros2 run rqt_image_view rqt_image_view
```

Used for:

* `/alphabot2/image_raw`

---

## 4. Change ROS network domain

```bash
ROS_DOMAIN_ID=12 ~/run_alphabot.sh
```
