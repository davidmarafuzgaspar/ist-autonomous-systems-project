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

You are now inside the container with ROS 2 ready.

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
