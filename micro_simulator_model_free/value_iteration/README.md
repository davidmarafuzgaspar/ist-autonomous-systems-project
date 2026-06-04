# Value iteration

Interactive **value iteration** on an oriented grid: configure the map, step through Bellman updates, inspect \(V^*\) and policy on the canvas.

## Flow

1. **Map setup** — grid, obstacles, start, goal, initial heading (arrow on start).
2. **VI viewer** — next step / run to convergence; Gauss–Seidel or Jacobi; edit \(\gamma\) and rewards; **Change world**.

```bash
cd micro_simulator_model_free
python -m value_iteration.main
```

| Flag | Description |
|------|-------------|
| `--skip-setup` | Open viewer with empty default grid |
| `--rows`, `--cols` | Size when setup is skipped (default 3×5) |

## Model (summary)

- **State** \((r,c,h)\); actions: straight, turn R/L/A (turn then forward step).
- **Deterministic** transitions; illegal move → stay, penalty; goal → terminal.
- Default \(\gamma=0.85\), goal +100, illegal −50, step/turn costs as in the viewer.

Full equations and reward table: same Bellman backup as in a standard tabular VI MDP.

## Files

| File | Role |
|------|------|
| `world.py` | MDP and `simulate_step` |
| `value_iteration.py` | VI sweeps |
| `grid_setup.py` | Map editor |
| `iteration_viewer.py` | GUI |
| `main.py` | Entry point |
