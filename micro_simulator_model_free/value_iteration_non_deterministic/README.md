# Value iteration (forward slip)

Same GUI and MDP as [`value_iteration/`](../value_iteration/README.md), with **slip** on the forward step (default 70% / 15% / 15% intended / left / right).

Planning uses the **expected** Bellman backup; edit slip % in the viewer (**Apply parameters**).

```bash
cd micro_simulator_model_free
python -m value_iteration_non_deterministic.main
```

Flags: same as deterministic (`--skip-setup`, `--rows`, `--cols`).

## Files

| File | Role |
|------|------|
| `world.py` | `transition_distribution`, slip weights |
| `value_iteration.py` | VI over expected \(Q\) |
| `grid_setup.py`, `iteration_viewer.py`, `main.py` | Same workflow as deterministic |
