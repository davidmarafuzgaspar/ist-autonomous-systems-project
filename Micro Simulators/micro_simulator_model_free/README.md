# Q-learning micro-simulator

Visualises the **model-free path from `solver.py`**: tabular Q-learning (ε-greedy, TD updates, ε decays per episode). Not the ROS robot — same algorithm on an editable grid.

## Run

Requires **numpy** and **tkinter**.

```bash
cd micro_simulator_model_free && python run.py
# or from repo root:
python -m micro_simulator_model_free
```

From inside the folder use `run.py`; `-m` only from the repo root.

## Workflow

1. **Map setup** — **5×5** grid by default; start, goal, heading, obstacles.
2. **Viewer** — Next step / Run episode / Train all; policy arrows (↑→↓←) only after training finishes.
3. **Change world** — back to setup (previous map pre-filled).

## Modules

| File     | Contents                                      |
| -------- | --------------------------------------------- |
| `model.py` | Oriented grid (`IntersectionWorld`) + `QLearningTrainer` |
| `gui.py`   | Map editor + `QLearningViewer`                |
| `main.py`  | Setup → viewer loop                           |
| `run.py`   | Launcher                                      |

Defaults: `α=0.2`, `γ=0.85`, `ε: 1.0 → 0.05` (decay `0.995`), `1000` episodes, `50` steps/episode.
