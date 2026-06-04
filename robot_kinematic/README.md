# Robot kinematic simulator

Continuous **AlphaBot2-style** simulator: differential drive on a line grid, **IR line sensors** and **IR obstacle rays**. Complements the discrete MDP micro-simulators; it does **not** run ROS.

## Run

Requires **tkinter** (no numpy).

```bash
cd robot_kinematic && python run.py
# or from repo root:
python -m robot_kinematic
```

## Workflow

1. **World setup** — grelha **5×5** por defeito; click nos cruzamentos para obstáculos.
2. **Simulation** — **WASD** / arrows to drive; **R** reset; **Change world** back to setup.

Sidebar shows `line_binary` and IR hits (same idea as `/alphabot2/ir_line_sensors` on hardware).

## Layout (3 modules)

| File | Role |
|------|------|
| `model.py` | Board, robot, obstacles, `AlphaBotSimulation`, `WorldSetup` |
| `gui.py` | World-setup dialog + WASD viewer (Tk theme inlined) |
| `main.py` | Setup → sim loop |
| `run.py` | Launcher when cwd is this folder |

## Relation to the real robot

- **ROS:** `alphabot2_ws/` — see root `README.md`.
- **MDP / planning:** `solver.py`, `micro_simulator_model_free/`, `micro_simulator_model_based/`.
