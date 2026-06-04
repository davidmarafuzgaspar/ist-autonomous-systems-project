# Micro-simulator (model-based)

Value iteration em grelha orientada (determinística ou com slip). Python 3, Tk, numpy.

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

| App                                  | Role                 |
| ------------------------------------ | -------------------- |
| `value_iteration/`                   | VI determinística    |
| `value_iteration_non_deterministic/` | VI com slip à frente |

Cada app: **map setup** (5×5 por defeito) → viewer VI → **Change world**.

## Layout (igual nos dois apps)

| Ficheiro   | Conteúdo                                         |
| ---------- | ------------------------------------------------ |
| `model.py` | MDP + `ValueIteration` + `rollout_greedy_policy` |
| `gui.py`   | Map setup + `InteractiveValueIterationViewer`    |
| `main.py`  | Loop setup → viewer                              |
| `run.py`   | Launcher                                         |
