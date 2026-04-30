# Guide for Alphabot control

## 1. Create Docker Launch Script

Create a reusable script so you don’t have to type everything every time.

```bash
nano ~/ros2_humble.sh
```

Paste the following (replace `ROS_DOMAIN_ID` with your robot ID if needed):

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

---

## 2. Make Script Executable

```bash
chmod +x ~/ros2_humble.sh
```

---

## 3. Connect to the Robot

---

### Terminal 1 — Launch main drivers

```bash
ssh deec@10.16.140.68
# password: deecrobots

ros2 launch alphabot2 alphabot2_launch.py
```

---

### Terminal 2 — Launch motion driver

```bash
ssh deec@10.16.140.68
# password: deecrobots

ros2 run alphabot2 motion_driver
```

---

## 4. Start Your Docker ROS 2 Environment

Open a **third terminal**:

```bash
~/ros2_humble.sh
```

You are now inside the container with ROS 2 ready.

---

## 🔍 5. Verify Connection

Inside the Docker container:

```bash
ros2 topic list
```