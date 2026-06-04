# Micro-simulator (model-based)

Python 3, Tk, numpy for VI.

**From inside each app folder** (works when your shell is already in `micro_simulator_model_based/`):

```bash
cd value_iteration && python run.py
cd value_iteration_non_deterministic && python run.py
cd kinematic && python run.py
```

**From the repository root**:

```bash
python -m micro_simulator_model_based.value_iteration.main
python -m micro_simulator_model_based.value_iteration_non_deterministic.main
python -m micro_simulator_model_based.kinematic.main
```

`python -m micro_simulator_model_based...` fails if the current directory is *inside* `micro_simulator_model_based/` — use `run.py` instead.

| Package | Role |
|---------|------|
| `value_iteration/` | Deterministic VI on an oriented grid |
| `value_iteration_non_deterministic/` | Same, with forward slip |
| `kinematic/` | Continuous board sim (line + IR sensors) |

Each app: **map/world setup** → main window → **Change world** reopens setup.

**Model-free Q-learning** lives in `../micro_simulator_model_free/` — see its README.

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
