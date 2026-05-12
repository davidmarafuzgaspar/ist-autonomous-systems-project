# Environment

This folder contains the first simulation assets for the AlphaBot2 environment.

## Current asset

- `worlds/alphabot2_cross_board_10x10.world`: square board with a 10 x 10 cross grid.
- `python_sim/`: pure Python modular simulator without Gazebo

## Pure Python simulator

The pure Python simulator is organized into small modules:

- `python_sim/config.py`: board and robot dimensions
- `python_sim/board.py`: board geometry and black/white line queries
- `python_sim/obstacles.py`: rectangular obstacles and range checks
- `python_sim/robot.py`: AlphaBot2 pose, commands and sensor sampling
- `python_sim/simulation.py`: full simulation state
- `python_sim/visualizer.py`: Tkinter viewer
- `python_sim/main.py`: entry point

Run the visual simulator:

```bash
python3 "Environment/python_sim/main.py"
```

Run a quick terminal-only test:

```bash
python3 "Environment/python_sim/main.py" --headless --steps 10
```

## Current robot setup

The same world now also includes a simplified `alphabot2` model with:

- chassis diameter: `15 cm`
- 2 front obstacle sensors with max range `7 cm`
- 5 bottom line sensors

## Sensor placement assumptions

- the 5 line sensors are placed `2 cm` behind the front edge of the robot
- the 5 line sensors are spaced `2 cm` apart from each other
- the line sensor row is centered laterally under the robot
- obstacle sensors are mounted at the front-left and front-right of the chassis
- obstacle sensor output in the Python simulator is `1 = obstacle detected`, `0 = free`

## Sensor behavior in simulation

- obstacle sensors are simulated as narrow front-facing range sensors with `0.07 m` max range
- line sensors are simulated as tiny downward-facing `1 x 1` cameras so they can "see" black vs white on the board
- if you want, the next step can convert these Gazebo sensor topics into the exact ROS messages used by your current nodes
- in the Python simulator, line sensors return both `binary` values and simple `analog` values

## Board assumptions used

- Road/line width: `4 cm` (`0.04 m`)
- Cross spacing: `15 cm` (`0.15 m`) center-to-center
- Number of crosses: `10` per row and `10` per column
- Total black-grid footprint: `1.39 m x 1.39 m`

The total footprint comes from:

- `9 * 0.15 m` between the first and last grid centers
- plus `0.04 m` for the line width

If by `15 x 15` you meant the free space between line edges instead of center-to-center spacing, the board dimensions should be updated. The current version is meant to be a clean starting point for the simulation world.
