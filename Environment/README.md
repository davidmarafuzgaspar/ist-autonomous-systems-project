# Environment

This folder contains the first simulation assets for the AlphaBot2 environment.

## Discrete MDP planner (`micro_sim`)

`micro_sim` models the 5x5 intersection grid as a finite Markov Decision
Process and solves it with Bellman value iteration. It is a planning
companion to `python_sim`: same grid, same obstacles, but discrete states
and actions so we can compute an optimal policy.

- `micro_sim/world.py`: `IntersectionWorld` (states, actions, deterministic
  transitions, reward shape, obstacle layout aligned with `python_sim`)
- `micro_sim/value_iteration.py`: Bellman value iteration solver with
  formula docstrings
- `micro_sim/display.py`: terminal pretty-print for layout, values and
  policy
- `micro_sim/viewer.py`: Tkinter viewer that renders the cross board
  identically to `python_sim` and overlays the policy arrow on every
  intersection
- `micro_sim/main.py`: entry point that prints the formula, runs the
  solver, displays `V*` and `pi*`, rolls the policy from `start` to
  `goal`, and finally launches the Tkinter viewer

Run the demo with the Tkinter viewer:

```bash
python3 "Environment/micro_sim/main.py"
```

Skip the viewer (terminal only):

```bash
python3 "Environment/micro_sim/main.py" --headless
```

Show `V*(s)` next to each arrow in the viewer:

```bash
python3 "Environment/micro_sim/main.py" --show-values
```

Other useful flags:

```bash
python3 "Environment/micro_sim/main.py" --gamma 0.9 --theta 1e-4 --verbose
```

The Bellman optimality equation used (deterministic transitions):

```
V*(s)  = max_a [ R(s, a, s') + gamma * V*(s') ]
pi*(s) = argmax_a [ R(s, a, s') + gamma * V*(s') ]
```

## Pure Python simulator

The pure Python simulator is organized into small modules:

- `python_sim/config.py`: board and robot dimensions
- `python_sim/board.py`: board geometry, black/white queries and white-cell marker layout
- `python_sim/obstacles.py`: rectangular obstacles and range checks
- `python_sim/robot.py`: AlphaBot2 pose, commands, sensor sampling and cell localization
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

## ArUco-style markers

- every interior white square contains one logical marker
- with `10` grid lines per axis, the board has `9 x 9 = 81` white squares and `81` markers
- marker IDs are assigned in row-major order, from top-left to bottom-right
- each marker sits at the center of its white square

## Camera localization

- the robot has one idealized camera mounted near the front-left of the chassis
- the camera looks slightly to the left of the robot forward direction
- detection is geometric only: range + field of view
- no image rendering or OpenCV detection is used in this version
- localization output is logical: `cell(row, col)` plus the marker ID used
- in `--headless` mode the simulator prints visible marker IDs and the localized cell

## Current robot setup

The simulator includes a simplified `alphabot2` model with:

- chassis diameter: `15 cm`
- 2 front obstacle sensors with max range `7 cm`
- 5 bottom line sensors
- 1 front-left idealized camera for marker-based localization

## Sensor placement assumptions

- the 5 line sensors are placed `2 cm` behind the front edge of the robot
- the 5 line sensors are spaced `2 cm` apart from each other
- the line sensor row is centered laterally under the robot
- obstacle sensors are mounted at the front-left and front-right of the chassis
- obstacle sensor output in the Python simulator is `1 = obstacle detected`, `0 = free`
- the camera is mounted near the front-left and points forward-left

## Sensor behavior in simulation

- obstacle sensors are simulated as narrow front-facing range sensors with `0.07 m` max range
- line sensors are sampled directly from the board black/white geometry
- the camera reads visible white-cell markers and localizes the robot to a logical cell
- in the Python simulator, line sensors return both `binary` values and simple `analog` values

## Board assumptions used

- Road/line width: `5.0 cm` (`0.05 m`)
- Cross spacing: `15 cm` (`0.15 m`) center-to-center
- Number of crosses: `10` per row and `10` per column
- Total black-grid footprint: `1.40 m x 1.40 m`

The total footprint comes from:

- `9 * 0.15 m` between the first and last grid centers
- plus `0.05 m` for the line width

If by `15 x 15` you meant the free space between line edges instead of center-to-center spacing, the board dimensions should be updated. The current version is meant to be a clean starting point for the simulation world.
