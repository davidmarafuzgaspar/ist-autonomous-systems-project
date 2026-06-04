# Micro-simulator (model-free)

Offline tools for the AlphaBot2 intersection grid: **tabular planning** (value iteration) and continuous **kinematic simulation**.

| Package | Command |
|---------|---------|
| [`value_iteration/`](value_iteration/README.md) | `python -m value_iteration.main` |
| [`value_iteration_non_deterministic/`](value_iteration_non_deterministic/README.md) | `python -m value_iteration_non_deterministic.main` (forward slip) |
| [`kinematic/`](kinematic/) | `python -m kinematic.main` |

```bash
cd micro_simulator_model_free
python -m value_iteration.main
python -m value_iteration_non_deterministic.main
python -m kinematic.main
```

Model details and formulas: see each VI package README.

---

## `value_iteration/` — deterministic VI

| File | Role |
|------|------|
| `__init__.py` | Package marker; entry docstring. |
| `main.py` | Entry point: map editor → viewer; **Change world** loop; flags `--skip-setup`, `--rows`, `--cols`. |
| `world.py` | Oriented-grid MDP: states \((r,c,h)\), S/R/L/A actions, `simulate_step`, rewards, Bellman \(Q\), per-cell optimal policy display. |
| `value_iteration.py` | Value iteration (Jacobi or Gauss–Seidel), stopping threshold \(\theta\), returns \(V^*\) and policy. |
| `grid_setup.py` | Tk window **before** VI: grid size, obstacles, start, goal, initial heading (arrow on start). |
| `iteration_viewer.py` | VI Tk viewer: step / run to convergence, \(V\) heatmap, policy arrows, edit \(\gamma\) and rewards, algorithm, **Change world**. |
| `ui_theme.py` | Shared Tk colors, fonts, and helpers for `grid_setup` and `iteration_viewer`. |

---

## `value_iteration_non_deterministic/` — VI with slip

Same layout as `value_iteration/`; file names and roles match except where the model is stochastic:

| File | Role |
|------|------|
| `__init__.py` | Package marker. |
| `main.py` | Same as deterministic (setup → viewer → **Change world**). |
| `world.py` | As deterministic, plus forward-step transition distribution (`transition_distribution`, intended/left/right weights) and **expected** Bellman backup. |
| `value_iteration.py` | VI over the expected backup (same API as the deterministic package). |
| `grid_setup.py` | Map editor (same as deterministic). |
| `iteration_viewer.py` | Viewer with slip probability controls and **Apply parameters**. |
| `ui_theme.py` | Tk theme (aligned with the deterministic package). |

---

## `kinematic/` — continuous board simulator

| File | Role |
|------|------|
| `main.py` | Entry: WASD GUI or `--headless` (prints sensors to the terminal). |
| `config.py` | Board (`BoardConfig`) and robot (`RobotConfig`) parameters: geometry, sensors, velocity limits. |
| `board.py` | Physical cross grid: white cells, ArUco markers, black-line detection under the robot. |
| `obstacles.py` | Rectangular obstacles; ray–rectangle and circle–rectangle tests for IR. |
| `robot.py` | AlphaBot2 differential drive, 2D pose, IR and line readings, marker-based localization. |
| `simulation.py` | Time-step integration: \((v,\omega)\) command, collisions, per-step `SensorSnapshot`. |
| `visualizer.py` | Tk canvas: board, robot, IR rays, status panel; keyboard driving. |
