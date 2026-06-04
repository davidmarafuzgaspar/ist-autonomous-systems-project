# Value iteration (stochastic forward)

Same oriented-grid MDP and UI as [`value_iteration/`](../value_iteration/README.md), with **perpendicular slip** on the forward displacement after each turn.

## Overview

| Stage | Tool |
|-------|------|
| 1. Map | `grid_setup` — identical workflow to the deterministic package |
| 2. Solve | `iteration_viewer` — VI plus slip weights (% intended / left / right) |
| 3. Batch (optional) | `main --final` — solve; rollouts with intended-only and sampled slip |

## Model (deterministic part)

States, actions, turn-then-forward semantics, rewards, and terminal goal handling match the deterministic package. See that README for the full action and reward tables.

## Slip model

After the turn, the robot attempts a **one-cell move** in one of three directions relative to the post-turn heading:

| Outcome | Default weight | After renormalization |
|---------|----------------|------------------------|
| Intended forward | 0.70 | 70% |
| Slip left (perpendicular) | 0.15 | 15% |
| Slip right (perpendicular) | 0.15 | 15% |

Weights are renormalized if their sum differs from 1. Turns are still deterministic; only the forward cell choice is random.

## Bellman backup (expectation)

\[
Q(s,a) = \sum_{s'} P(s'|s,a)\,\bigl[ R(s,a,s') + \gamma V(s') \bigr],
\qquad
V_{k+1}(s) \leftarrow \max_a Q_k(s,a).
\]

Value iteration uses this expectation (no sampling during planning). Execution rollouts can use either:

- **Intended-only** — always take the intended forward branch (matches deterministic physics).
- **Stochastic** — sample from \(P(s'|s,a)\) (`sample_transition`).

## Usage

```bash
cd micro_simulator_model_free
python -m value_iteration_non_deterministic.main
```

| Flag | Description |
|------|-------------|
| `--final` | Terminal solve; intended and stochastic rollouts |
| `--skip-setup` | Skip the map editor |
| `--rows`, `--cols` | Grid size when setup is skipped |
| `--slip-intended`, `--slip-left`, `--slip-right` | Slip weights (renormalized) |
| `--seed` | RNG seed for stochastic rollout |

**GUI:** edit rewards and slip percentages, then **Apply parameters**. **Change world** preserves the last map and parameters.

## Module reference

| File | Role |
|------|------|
| `world.py` | `transition_distribution`, `bellman_action_value`, `sample_transition` |
| `value_iteration.py` | VI sweeps over expected \(Q\) |
| `main.py` | CLI |
| `grid_setup.py`, `iteration_viewer.py`, `display.py`, `ui_theme.py` | Shared structure with deterministic VI |
