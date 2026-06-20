# IST Autonomous Systems Project

This repository contains the complete project for the Instituto Superior Técnico Autonomous Systems course.

## Authors

- David Marafuz Gaspar — 106541
- Miguel Veiga Carneiro — 106218
- Francisco Parro Valério — 106533
- Pedro Gaspar Mónico — 106626

## Project Overview

This project implements autonomous grid navigation on the Waveshare AlphaBot2 mobile robot using line following, discrete intersection handling, and MDP-based planning. The work spans offline simulation, ROS 2 hardware integration, and on-robot execution with three solver modes:

- **Model-based** — full map knowledge, value iteration policy
- **Model-free** — ground-truth map, Q-learning policy (offline)
- **Dynamic** — partial map belief, Q-learning at startup, runtime obstacle discovery with value-iteration replanning

## Repository Structure

```
ist-autonomous-systems-project/
├── Controller/                  # On-robot navigation stack
│   ├── main.py                  # ROS 2 line follower + junction FSM
│   ├── solver.py                # Q-learning, value iteration, three modes
│   └── world.py                 # Discrete grid world
│
├── ROS2 Packages/               # Robot drivers and interfaces
│   ├── alphabot2/               # Motion, IR line/obstacle sensors, launch
│   ├── alphabot2_interfaces/    # Custom messages (Obstacle, etc.)
│   └── docs/                    # Package documentation
│
├── Micro Simulators/            # Desktop MDP and kinematic simulators
│   ├── micro_simulator_model_free/
│   ├── micro_simulator_model_based/
│   ├── micro_simulator_dynamic/
│   └── robot_kinematic_simulator/
│
├── Tutorials/                   # Step-by-step setup guides
│   ├── 1_Environment_Setup.md
│   ├── 2_Camera_Calibration.md
│   ├── 3_Node_Creation.md
│   └── 4_Home_Wifi_Setup.md
│
├── ROS2 Bags/                   # Recorded sensor data and plotting scripts
├── Videos/                      # Demonstration run documentation
├── Presentations/               # Project presentations
│
├── Assignment.pdf               # Project specification
├── Guide.pdf                    # Course / robot guide
└── Report.pdf                   # Project report
```

## Controller

**Location**: `Controller/`

The runtime navigation node combines reflective line sensing with a junction state machine (FOLLOW → ALIGN → SEARCH) and a grid-world pose tracker. The solver selects actions at intersections according to the configured mode.

**Key features**:

- Five IR line sensors (binary thresholding, weighted P+D line follow)
- Junction detection and in-place search for branch alignment
- Three solver modes (`model_based`, `model_free`, `dynamic`)
- IR obstacle sensing in dynamic mode with map updates and replanning

**Run on robot** (after ROS workspace is built and sourced):

```bash
python3 main.py
```

## ROS 2 Packages

**Location**: `ROS2 Packages/`

Hardware-facing nodes for the AlphaBot2 on Raspberry Pi: motor driver, IR line sensors (TLC1543), IR obstacle sensors (ST188), and launch configuration.

**Key topics**:

| Topic | Description |
|-------|-------------|
| `/alphabot2/cmd_vel` | Velocity commands |
| `/alphabot2/ir_line_sensors` | Raw line sensor ADC values |
| `/alphabot2/ir_obstacles_sensors` | Left/right obstacle flags |

**Launch on robot**:

```bash
ros2 launch alphabot2 alphabot2_launch.py force_obstacle_stop:=true
```

**See**: [ROS2 Packages docs](ROS2%20Packages/docs/README.md)

## Micro Simulators

**Location**: `Micro Simulators/`

Offline tools for developing and validating planning logic before deployment on the robot.

| Folder | Purpose |
|--------|---------|
| `micro_simulator_model_free/` | Tabular Q-learning on a grid |
| `micro_simulator_model_based/value_iteration/` | Deterministic value iteration |
| `micro_simulator_model_based/value_iteration_non_deterministic/` | Value iteration with slip |
| `micro_simulator_dynamic/` | Known vs hidden map, sense and replan |
| `robot_kinematic_simulator/` | Continuous line + IR sensor simulation |

Run from each folder: `python run.py` (see individual README files).

## Tutorials

**Location**: `Tutorials/`

Step-by-step guides for environment setup and robot configuration:

- [1 — Environment Setup](Tutorials/1_Environment_Setup.md) — Ubuntu 24.04, Docker, ROS 2 workspace, Foxglove
- [2 — Camera Calibration](Tutorials/2_Camera_Calibration.md) — Calibration file and launch configuration
- [3 — Node Creation](Tutorials/3_Node_Creation.md) — IR line sensor node, build, deploy
- [4 — Home Wi-Fi Setup](Tutorials/4_Home_Wifi_Setup.md) — NetworkManager profiles via SD card

## Requirements

**On-robot (Raspberry Pi)**:

- ROS 2 Humble
- Python 3, `rclpy`, `RPi.GPIO`
- AlphaBot2 hardware stack

**Development PC**:

- Ubuntu 24.04 (recommended)
- Docker, Foxglove Studio
- Python 3 with `numpy` (Controller and simulators)

Detailed install steps: [Tutorial 1](Tutorials/1_Environment_Setup.md).

## Documentation

- [Environment setup](Tutorials/1_Environment_Setup.md)
- [Camera calibration](Tutorials/2_Camera_Calibration.md)
- [Line sensor node](Tutorials/3_Node_Creation.md)
- [Home Wi-Fi](Tutorials/4_Home_Wifi_Setup.md)
- [Demo video notes](Videos/README.md)
- [Model-free simulator](Micro%20Simulators/micro_simulator_model_free/README.md)
- [Model-based simulator](Micro%20Simulators/micro_simulator_model_based/README.md)
- [Dynamic simulator](Micro%20Simulators/micro_simulator_dynamic/README.md)
- [Kinematic simulator](Micro%20Simulators/robot_kinematic_simulator/README.md)

## Important Notes

- Hardware nodes run on the robot; Docker on the PC is used for visualization and development.
- Match `ROS_DOMAIN_ID` between robot and PC for topic discovery.
- Dynamic mode is the only mode that processes IR obstacles at runtime and replans with value iteration.
- Wi-Fi credentials in Tutorial 4 should not be committed to Git.

---

*IST Autonomous Systems course project — academic year 2025–2026.*
