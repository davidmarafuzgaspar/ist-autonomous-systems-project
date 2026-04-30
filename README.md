Here’s your updated **README section**, now including Docker installation (Ubuntu 24 + Windows) and the ROS image setup, cleanly integrated:

---

# Docker + ROS 2 Humble Setup

## Install Docker

### Ubuntu 24.04

```bash
sudo apt update
sudo apt install docker.io -y
sudo systemctl enable --now docker
```

---

### Windows

Install Docker Desktop:

1. Download from the official website
2. Install and open Docker Desktop
3. Enable **WSL 2 integration** when prompted

---

## Step 1 — Pull the ROS 2 Humble Image

```bash
docker pull osrf/ros:humble-desktop-full
```

---

## Step 2 — Create a Launch Script

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

## Robot Connection & Operation

### Terminal 1 — Launch main drivers

```bash
ssh deec@10.16.140.68
```

```bash
ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — Launch motion driver

```bash
ssh deec@10.16.140.68
```

```bash
ros2 run alphabot2 motion_driver
```

---

## 4. Start Your Docker ROS 2 Environment

Open a **third terminal**:

```bash
~/ros2_humble.sh
```

You are now inside the Docker container with ROS 2 ready.

---

## 🔍 5. Verify Connection

Inside the Docker container:

```bash
ros2 topic list
```

You should see active topics from the robot.

---

## 6. Basic Control Commands

### Publish velocity commands

```bash
ros2 topic pub --rate 1 /alphabot2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

---

### Keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### View camera feed

```bash
ros2 run rqt_image_view rqt_image_view
```
