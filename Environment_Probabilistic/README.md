# Environment_Probabilistic

Copy of `Environment` where **translation** in the discrete MDP is **stochastic**
(slip left/right with configurable probabilities). The original `Environment`
folder is unchanged.

## Discrete MDP planner (`micro_sim`)

Same 5x5 grid and rewards as `Environment/python_sim`. **Bellman backups**
use the expectation over next states:

\[
Q(s,a) = \sum_{s'} P(s'|s,a)\bigl[R(s,a,s') + \gamma V(s')\bigr],\quad
V(s) = \max_a Q(s,a).
\]

**Rollouts** still sample one stochastic successor per step (see `--seed`).

- `micro_sim/world.py`: `IntersectionWorld` with `motion_prob_forward` /
  `motion_prob_left` / `motion_prob_right` (non-negative, renormalized to sum
  to 1). **Rotations** (`L`/`R` in oriented mode) stay deterministic.
- `micro_sim/value_iteration.py`: uses `transition_distribution` for the
  expectation above
- Other modules mirror `Environment/micro_sim` (display, viewers, analysis, main)

Run from this folder:

```bash
cd Environment_Probabilistic && python -m micro_sim.main --headless
```

Oriented MDP (default in `--interactive`):

```bash
cd Environment_Probabilistic && python -m micro_sim.main --oriented --headless
```

Adjust slip weights from the CLI:

```bash
python -m micro_sim.main --oriented --headless --p-forward 0.7 --p-left 0.15 --p-right 0.15 --seed 0
```

Show `V*(s)` next to each arrow in the viewer:

```bash
cd Environment_Probabilistic && python -m micro_sim.main --show-values
```

Launch the interactive iteration viewer (step through with a button):

```bash
cd Environment_Probabilistic && python -m micro_sim.main --interactive
```

Other useful flags:

```bash
cd Environment_Probabilistic && python -m micro_sim.main --gamma 0.9 --theta 1e-4 --verbose
cd Environment_Probabilistic && python -m micro_sim.main --trace
cd Environment_Probabilistic && python -m micro_sim.main --sweep step-cost
```

The Bellman optimality equation used here (stochastic translation, expectation over `s'`):

```
Q(s,a) = sum_{s'} P(s'|s,a) [ R(s,a,s') + gamma * V*(s') ]
V*(s)  = max_a Q(s,a)
pi*(s) = argmax_a Q(s,a)
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
cd Environment_Probabilistic && python3 python_sim/main.py
```

Run a quick terminal-only test:

```bash
cd Environment_Probabilistic && python3 python_sim/main.py --headless --steps 10
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
