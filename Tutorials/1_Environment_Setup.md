# Tutorial 1 — Environment Setup (Ubuntu 24.04 + Docker)

This tutorial covers the one-time setup on your development PC: Docker, the ROS 2 Humble container, Foxglove, and the AlphaBot2 workspace layout.

---

## Prerequisites

- Ubuntu 24.04 on your laptop/PC
- Network access to pull Docker images and clone repositories

---

## Step 1 — Install Docker

```bash
sudo apt update
sudo apt install docker.io -y
sudo systemctl enable --now docker
```

Add your user to the `docker` group so you can run containers without `sudo` every time:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Step 2 — Pull the base ROS 2 image

```bash
docker pull osrf/ros:humble-desktop-full
```

---

## Step 3 — Create the Docker build folder

```bash
mkdir ~/alphabot2_docker
cd ~/alphabot2_docker
nano Dockerfile
```

Paste the following Dockerfile:

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

## Step 4 — Build the Docker image (one time)

```bash
cd ~/alphabot2_docker
docker build -t alphabot2_ros2 .
```

---

## Step 5 — Create the runtime launcher script

```bash
nano ~/run_alphabot2.sh
```

Paste:

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

Make it executable:

```bash
chmod +x ~/run_alphabot2.sh
```

---

## Step 6 — Install Foxglove Studio (host machine)

```bash
sudo snap install foxglove-studio
```

---

## Step 7 — Create the ROS 2 workspace

All packages must live under `src`:

```bash
mkdir -p ~/alphabot2_ws/src
```

Expected layout:

```text
~/alphabot2_ws/
 └── src/
```

---

## Step 8 — Add robot source code

### Option A (recommended): Clone from Git

```bash
cd ~/alphabot2_ws/src
git clone https://github.com/Mik3Rizzo/alphabot2-ros2.git
```

### Option B: Copy from the robot (one-time fallback)

```bash
scp -r deec@ROBOT_IP:~/alphabot2_ws/src ~/alphabot2_ws/
```

Replace `ROBOT_IP` with the robot address on your network.

---

## Step 9 — Final workspace structure

After cloning or copying, you should have:

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

## Step 10 — Start a development session

**On the robot** (SSH), launch hardware nodes:

```bash
ssh deec@ROBOT_IP
ros2 launch alphabot2 alphabot2_launch.py force_obstacle_stop:=true
```

**On your PC**, start Docker (match `ROS_DOMAIN_ID` to the robot):

```bash
ROS_DOMAIN_ID=68 ~/run_alphabot2.sh
```

Inside the container, build if code changed:

```bash
cd /alphabot2_ws
colcon build
source install/setup.bash
```

Start the Foxglove bridge:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

Expected output:

```text
Server listening on port 8765
```

In Foxglove Studio on the host: WebSocket → `ws://localhost:8765`

Verify topics:

```bash
ros2 topic list
```

---

## Key rules

| Item | Location |
|------|----------|
| Git repo | `~/alphabot2_ws/src/` |
| Docker mount | Whole `~/alphabot2_ws` |
| Hardware nodes | Run on the robot |
| Visualization / bridge | Run in Docker on PC |

---

## Troubleshooting

- **Docker permission denied:** Log out and back in after `usermod -aG docker`, or run `newgrp docker`.
- **No ROS topics in Docker:** Check `ROS_DOMAIN_ID` matches the robot.
- **Foxglove cannot connect:** Confirm `foxglove_bridge` is running and port 8765 is not blocked.
