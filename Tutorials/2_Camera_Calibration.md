# Tutorial 2 — Camera Calibration

This tutorial explains how to calibrate the AlphaBot2 camera, save the calibration file, copy it to the robot, and point the launch file at it.

---

## Goal

1. Run camera calibration and produce `ost.yaml`
2. Copy the result to the robot as `camera_info.yaml`
3. Enable the camera node in the launch file with the correct path
4. Rebuild and verify

---

## Step 1 — Run calibration on your PC

Use the standard ROS camera calibration tool against the robot camera stream (or a recorded bag). When calibration finishes, the tool writes output to:

```bash
cat /tmp/ost.yaml
```

Keep this file; it contains intrinsics and distortion coefficients for your specific camera.

---

## Step 2 — Copy calibration to the robot

Replace `ROBOT_IP` and user if needed:

```bash
scp /tmp/ost.yaml deec@ROBOT_IP:~/camera_info.yaml
```

On the robot, confirm the file exists:

```bash
ssh deec@ROBOT_IP
cat ~/camera_info.yaml
```

---

## Step 3 — Update the launch file

On the robot (or in your repo before deploy), edit:

```bash
nano ~/alphabot2_ws/src/alphabot2-ros2/alphabot2/launch/alphabot2_launch.py
```

Uncomment the `v4l2_camera_node` block and set parameters like this:

```python
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

Also add `v4l2_camera_node` to the `LaunchDescription([...])` list at the bottom of the file.

Important:

- Use `file:///home/deec/camera_info.yaml` (three slashes after `file:`).
- Adjust the username in the path if your robot user is not `deec`.

---

## Step 4 — Rebuild on the robot

```bash
cd ~/alphabot2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alphabot2
source install/setup.bash
```

---

## Step 5 — Launch and verify

```bash
ros2 launch alphabot2 alphabot2_launch.py force_obstacle_stop:=true
```

In another terminal:

```bash
ros2 topic list | grep image
ros2 topic echo /alphabot2/camera_info --once
```

Optional — view the image in Docker on your PC:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic: `/alphabot2/image_raw`

---

## Step 6 — Re-calibrate when needed

Re-run calibration if you:

- Replace the camera module
- Change resolution (currently 320×240 in launch)
- See strong lens distortion in downstream nodes (e.g. QR detection)

After re-calibration, repeat Steps 2–5.

---

## Troubleshooting

| Problem | Check |
|---------|--------|
| `camera_info_url` not found | Path and `file://` prefix; file owned by robot user |
| No image topic | `v4l2_camera_node` uncommented in launch; USB camera connected |
| Wrong FOV / scale | Re-run calibration; confirm `image_size` matches calibration resolution |
