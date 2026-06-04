# Value iteration (deterministic)

Interactive **value iteration** on an oriented grid MDP: configure the map, step through Bellman updates, then inspect \(V^*\) and \(\pi^*\).

## Overview

| Stage | Tool |
|-------|------|
| 1. Map | `grid_setup` — rows/cols, obstacles, start, goal, initial heading |
| 2. Solve | `iteration_viewer` — step or run VI; edit \(\gamma\) and rewards |
| 3. Batch (optional) | `main --final` — full solve and terminal rollout without GUI |

## Model

**State** \(s = (r, c, h)\): grid cell \((r,c)\), heading \(h \in \{N,E,S,W\}\) (row 0 is north).

**Actions** (turn, then one forward step along the heading after the turn):

| Action | Effect |
|--------|--------|
| Straight | No turn |
| Turn right / left | Quarter turn |
| Turn around | Half turn |

**Transitions** are deterministic:

- Blocked or out-of-bounds forward step → remain at \((r,c,h)\), reward \(r_{\text{illegal}}\).
- Enter goal → \((r',c',h')\), reward \(r_{\text{goal}}\), terminal.
- Otherwise → move to \((r',c',h')\), action cost \(R(a)\).

Goal poses are terminal with \(V = 0\).

## Default rewards

| Parameter | Default |
|-----------|---------|
| \(\gamma\) | 0.85 |
| \(r_{\text{goal}}\) | +100 |
| \(r_{\text{illegal}}\) | −50 |
| Straight | −1 |
| Turn right / left | −5 each |
| Turn around | −10 |

Editable in the viewer via **Apply rewards**.

## Bellman optimality

\[
V^*(s) = \max_a \Big[ R(s,a) + \gamma \sum_{s'} P(s'|s,a)\, V^*(s') \Big],
\qquad
\pi^*(s) = \arg\max_a Q(s,a).
\]

Here \(P(s'|s,a) = 1\) on the single successor \(s' = f(s,a)\). One iteration:

\[
V_{k+1}(s) \leftarrow \max_a \bigl[ R(s,a) + \gamma V_k(f(s,a)) \bigr].
\]

Stop when \(\max_s |V_{k+1}(s) - V_k(s)| < \theta\) (default \(10^{-6}\)) or after 200 iterations.

## Update rules (viewer)

| Mode | Behaviour |
|------|-----------|
| **Gauss–Seidel** | In-place updates; each backup uses the latest \(V_{k+1}\) where available |
| **Jacobi** | All backups read from a snapshot \(V_k\), then commit |

Both methods converge to the same \(V^*\) and \(\pi^*\); per-iteration values and iteration counts may differ.

## Usage

```bash
cd micro_simulator_model_free
python -m value_iteration.main
```

| Flag | Description |
|------|-------------|
| `--final` | Solve in the terminal; print layout, \(V^*\), policy, rollout |
| `--skip-setup` | Skip the map editor |
| `--rows`, `--cols` | Grid size when setup is skipped (default 3×5) |

**GUI flow:** Map setup → VI viewer → **Change world** (reopens setup with the previous configuration).

Grid size per axis: 2–12 cells.

## Module reference

| File | Role |
|------|------|
| `world.py` | `IntersectionWorld`, `simulate_step`, Bellman \(Q\) |
| `value_iteration.py` | VI sweeps (Jacobi / Gauss–Seidel) |
| `main.py` | CLI |
| `grid_setup.py` | Initial map dialog |
| `iteration_viewer.py` | Interactive canvas + sidebar |
| `display.py` | ASCII map / values / policy |
