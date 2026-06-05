# Micro-simulator (model-based)

Grid-based value iteration (deterministic or with forward slip). Python 3, Tk, numpy.

## Run

```bash
cd value_iteration && python run.py
cd value_iteration_non_deterministic && python run.py
```

From repo root:

```bash
python -m micro_simulator_model_based.value_iteration
python -m micro_simulator_model_based.value_iteration_non_deterministic
```

Inside `micro_simulator_model_based/` use `run.py` in each subfolder (not `-m`).

| App                                  | Role             |
| ------------------------------------ | ---------------- |
| `value_iteration/`                   | VI deterministic |
| `value_iteration_non_deterministic/` | VI with slip     |

Each app: **map setup** (5×5 by default) → VI viewer → **Change world**.

## Layout (same in both apps)

| File     | Contents                                         |
| -------- | ------------------------------------------------ |
| `model.py` | MDP + `ValueIteration` + `rollout_greedy_policy` |
| `gui.py`   | Map setup + `InteractiveValueIterationViewer`    |
| `main.py`  | Setup → viewer loop                              |
| `run.py`   | Launcher                                       |
