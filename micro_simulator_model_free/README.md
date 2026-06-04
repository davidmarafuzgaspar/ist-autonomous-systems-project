# Micro-simulator (model-free)

Offline planners and simulators for the AlphaBot2 intersection grid MDP.

| Package | Role |
|---------|------|
| `value_iteration/` | Map editor + interactive value iteration (Gauss–Seidel / Jacobi) |
| `value_iteration_non_deterministic/` | Stochastic F/L/R turn model |
| `environment/` | Continuous grid simulator (geometry, sensors) |

## Value iteration

```bash
cd micro_simulator_model_free
python -m value_iteration.main
```

Model, Bellman equations, and flags: `value_iteration/README.md`.
