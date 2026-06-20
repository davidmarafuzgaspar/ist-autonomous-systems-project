# Tutorial 3 — Line Sensor Node (IR_line_sensors)

This tutorial walks through adding and running the IR line sensor node: Python node, `setup.py` entry point, launch file, build, and topic verification.

The AlphaBot2 has five reflective IR line sensors read through a TLC1543 ADC. The node publishes raw values on `/alphabot2/ir_line_sensors`.

---

## Goal

1. Understand where the node lives in the package
2. Register it in `setup.py`
3. Include it in the launch file
4. Build, run, and verify the topic

---

## Step 1 — Repository layout

Relevant paths (adjust if your clone differs):

```text
alphabot2_ws/src/alphabot2-ros2/
 ├── alphabot2/
 │   ├── alphabot2/
 │   │   └── IR_line_sensors.py    ← node implementation
 │   ├── launch/
 │   │   └── alphabot2_launch.py
 │   └── setup.py                  ← entry points
 └── alphabot2_interfaces/
```

Or in this project repo:

```text
ROS2 Packages/alphabot2/
 ├── alphabot2/IR_line_sensors.py
 ├── launch/alphabot2_launch.py
 └── setup.py
```

---

## Step 2 — Node overview (`IR_line_sensors.py`)

The node:

- Reads 5 ADC channels via GPIO (TLC1543)
- Publishes `std_msgs/msg/Int32MultiArray` on topic `ir_line_sensors`
- Runs at 40 Hz (25 ms timer)
- Message format: `data = [sensor1, sensor2, sensor3, sensor4, sensor5]` (far-left to far-right)

Topic with namespace: `/alphabot2/ir_line_sensors`

---

## Step 3 — Register the entry point in `setup.py`

In `setup.py`, under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        'motion_driver = alphabot2.motion_driver:main',
        'IR_obstacle_sensors = alphabot2.IR_obstacle_sensors:main',
        'IR_line_sensors = alphabot2.IR_line_sensors:main',
    ],
},
```

The left side (`IR_line_sensors`) is the executable name used with `ros2 run`.

---

## Step 4 — Add the node to the launch file

In `alphabot2_launch.py`:

```python
ir_line_sensors_node = Node(
    package="alphabot2",
    namespace=NAMESPACE,
    executable="IR_line_sensors",
    output="screen",
    emulate_tty=True,
    arguments=['--ros-args', '--log-level', 'WARN'],
)

return LaunchDescription([
    force_obstacle_stop_arg,
    motion_driver_node,
    ir_obstacle_sensors_node,
    ir_line_sensors_node,
])
```

---

## Step 5 — Build on the robot (or locally for syntax check)

```bash
cd ~/alphabot2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alphabot2_interfaces --allow-overriding alphabot2_interfaces
colcon build --packages-select alphabot2 --allow-overriding alphabot2
source install/setup.bash
```

---

## Step 6 — Run the node alone (optional test)

```bash
ros2 run alphabot2 IR_line_sensors
```

Expected log (values change with surface):

```text
line_sensors=[712, 450, 380, 445, 698]
```

Press `Ctrl+C` to stop.

---

## Step 7 — Run via launch file (normal use)

```bash
ros2 launch alphabot2 alphabot2_launch.py force_obstacle_stop:=true
```

---

## Step 8 — Verify the topic

In a second terminal on the robot:

```bash
source ~/alphabot2_ws/install/setup.bash
ros2 topic list | grep line
ros2 topic echo /alphabot2/ir_line_sensors
```

You should see five integer values per message, updating continuously.

---

## Step 9 — Deploy from your PC to the robot

Set connection variables:

```bash
ROBOT_USER=deec
ROBOT_IP=192.168.1.150
ROBOT_WS=/home/$ROBOT_USER/alphabot2_ws
```

Sync the package:

```bash
rsync -av --delete \
  ~/path/to/project/ROS2\ Packages/alphabot2/ \
  ${ROBOT_USER}@${ROBOT_IP}:${ROBOT_WS}/src/alphabot2-ros2/alphabot2/
```

Then SSH to the robot and repeat Step 5.

---

## Adding another node (same pattern)

1. Create `alphabot2/your_node.py` with a `main()` function
2. Add `'your_node = alphabot2.your_node:main'` to `setup.py`
3. Add a `Node(...)` block in `alphabot2_launch.py`
4. `colcon build --packages-select alphabot2`
5. `ros2 run alphabot2 your_node` or launch with the full stack

---

## Troubleshooting

| Problem | Check |
|---------|--------|
| `Package not found` | `source install/setup.bash` after build |
| No topic | Node running on robot (GPIO needs Raspberry Pi hardware) |
| Wrong topic name | Namespace `alphabot2` → full name `/alphabot2/ir_line_sensors` |
| Permission on GPIO | Run on robot OS, not inside Docker on PC |
