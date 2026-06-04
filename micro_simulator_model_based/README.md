# Micro-simulator (model-free)

Run from this directory (Python 3, Tk):

```bash
cd micro_simulator_model_free
python -m value_iteration.main
python -m value_iteration_non_deterministic.main
python -m kinematic.main
```

| Package | Role |
|---------|------|
| `value_iteration/` | Deterministic VI on an oriented grid |
| `value_iteration_non_deterministic/` | Same, with forward slip |
| `kinematic/` | Continuous board sim (line + IR sensors) |

Each app: **map/world setup** → main window → **Change world** reopens setup.

## `value_iteration/`

| File | Role |
|------|------|
| `main.py` | Entry loop |
| `world.py` | MDP, `simulate_step`, Bellman backup |
| `value_iteration.py` | Jacobi / Gauss–Seidel sweeps |
| `grid_setup.py` | Map editor |
| `iteration_viewer.py` | VI GUI |
| `ui_theme.py` | Tk styling |

## `value_iteration_non_deterministic/`

Same files as above; `world.py` adds slip on the forward step; viewer edits slip weights.

## `kinematic/`

| File | Role |
|------|------|
| `main.py` | Entry loop |
| `world_setup.py` | Lines×columns grid, obstacles on junctions |
| `board.py` | Cross geometry, line sampling |
| `config.py` | Board and robot parameters |
| `obstacles.py` | Rectangular obstacles, IR ray tests |
| `robot.py` | Differential drive and sensors |
| `simulation.py` | Time stepping |
| `visualizer.py` | WASD GUI |
| `ui_theme.py` | Tk styling |
