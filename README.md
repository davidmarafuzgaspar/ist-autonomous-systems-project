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
mkdir ~/alphabot2_docker
cd ~/alphabot2_docker
nano Dockerfile
```

### Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt-image-view \
    git \
    python3-colcon-common-extensions \
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
nano ~/run_alphabot2.sh
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
chmod +x ~/run_alphabot2.sh
```

---

## 6. Install Foxglove Studio (host machine)

```bash
sudo snap install foxglove-studio
```

---

# 2. Workspace Setup (ROS 2 Development Workspace)

---

## 2.1 Create workspace directory

All ROS packages MUST live inside the `src` folder:

```bash
mkdir -p ~/alphabot2_ws/src
```

Result:

```text
~/alphabot2_ws/
 └── src/
```

---

## 2.2 Add your robot source code (IMPORTANT)

You now have TWO valid options.

---

### Option A (recommended): Clone from Git

This is the correct long-term workflow.

```bash
cd ~/alphabot2_ws/src
git clone https://github.com/Mik3Rizzo/alphabot2-ros2.git
```

Result:

```text
~/alphabot2_ws/src/
 └── alphabot2-ros2/
     ├── alphabot2
     ├── alphabot2_interfaces
     ├── docs
```

---

### Option B (only once): Copy from robot

Use only if Git is not available:

```bash
scp -r deec@10.16.140.68:~/alphabot2_ws/src ~/alphabot2_ws/
```

---

## 2.3 Final workspace structure

```text
~/alphabot2_ws/
 ├── src/
 │   └── alphabot2-ros2/
 │       ├── alphabot2
 │       ├── alphabot2_interfaces
 ├── build/
 ├── install/
 ├── log/
```

---

# 3. Usage (Every session workflow)

---

## 1. Start robot system (SSH)

### Terminal 1 — main robot drivers

```bash
ssh deec@10.16.140.68
```

```bash
ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — motion driver

```bash
ssh deec@10.16.140.68
```

```bash
ros2 run alphabot2 motion_driver
```

---

## 2. Start ROS Docker environment (local machine)

### Terminal 3

```bash
ROS_DOMAIN_ID=68 ~/run_alphabot2.sh
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

Expected output:

```
Server listening on port 8765
```

---

## 5. Connect Foxglove Studio (host machine)

* Open Foxglove Studio
* Select WebSocket connection
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
ROS_DOMAIN_ID=68 ~/run_alphabot2.sh
```

---

# Key rule to remember

* Git repo goes in: `~/alphabot2_ws/src/`
* Docker mounts: whole `~/alphabot2_ws`
* Robot runs hardware nodes
* Docker processes and visualizes data

---
# Calibração da Câmara

O ficheiro da calibração está aqui no meu PC:
```bash
cat /tmp/ost.yaml
```

Para passar para o alphabot2 tem de se correr (NÃO ESQUECER DE MUDAR O ID DO ALPHABOT2):
```bash
scp /tmp/ost.yaml deec@10.16.140.68:~/camera_info.yaml
```

depois modificar o lauch file no alphabot2:
```bash
nano ~/alphabot2-ros2/alphabot2/launch/alphabot2_launch.py
```
Deve ser alterado para:

```bash
v4l2_camera_node = Node(
    package="v4l2_camera",
    namespace=NAMESPACE,
    executable="v4l2_camera_node",
    output="screen",
    emulate_tty=True,
    arguments=['--ros-args',
               '--log-level', V4L2_CAMERA_LOG_LVL],
    parameters=[{
        'image_size': [320, 240],
        'camera_info_url': 'file:///home/deec/camera_info.yaml',
    }],
)
```

Concluir com dar colcon build:
```bash
cd ~/alphabot2-ros2
colcon build --packages-select alphabot2
source install/setup.bash
```
---

