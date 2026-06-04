# Value iteration (forward slip)

```bash
# from this folder
python run.py

# or from repository root
python -m micro_simulator_model_based.value_iteration_non_deterministic.main
```

Same viewer workflow as [`value_iteration/`](../value_iteration/README.md): map setup → VI steps → **per-heading** V and policy arrows **after converge** → green rollout path.

Forward slip default 70% / 15% / 15% (intended / left / right); edit rewards and slip % in the viewer.
