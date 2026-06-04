# Micro-simulator (model-free)

Offline tools for planning on a discrete **intersection grid** with orientation. Intended for analysis and demos before deployment on the physical AlphaBot2 stack.

## Packages

| Directory | Description |
|-----------|-------------|
| [`value_iteration/`](value_iteration/README.md) | Deterministic MDP; map editor; interactive value iteration |
| [`value_iteration_non_deterministic/`](value_iteration_non_deterministic/README.md) | Same MDP + stochastic perpendicular slip on forward motion |
| [`environment/`](environment/) | Continuous kinematic simulator (board geometry, sensors) |

## Quick start

```bash
cd micro_simulator_model_free

# Deterministic planning
python -m value_iteration.main

# Planning with forward slip
python -m value_iteration_non_deterministic.main
```

Full model definitions, equations, and CLI flags are documented in each package README.

## Layout

Each VI package is a self-contained Python package:

- `world.py` — state space, transitions, rewards (and slip probabilities in the non-deterministic variant)
- `value_iteration.py` — Bellman backup sweeps
- `main.py` — entry point (`python -m …`)
- `grid_setup.py`, `iteration_viewer.py`, `display.py`, `ui_theme.py` — map editor and Tk viewer

Run from `micro_simulator_model_free` so that `python -m package.main` resolves imports correctly.
