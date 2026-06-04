# Value iteration

Tabular **value iteration** on an oriented grid MDP for the AlphaBot2 intersection layout.

```bash
cd micro_simulator_model_free
python -m value_iteration.main
```

Map setup → step through VI or run to convergence → **Change world**.

---

## MDP

- **State** \(s = (r, c, h)\): cell \((r,c)\) on the grid and heading \(h \in \{N,E,S,W\}\). Obstacle cells are not states.
- **Actions** \(a \in \{\text{straight}, \text{turn R}, \text{turn L}, \text{turn A}\}\): apply the turn (if any), then move one cell forward along the new heading.
- **Transitions** are deterministic: `simulate_step` returns \((s', r, \text{done})\). Hitting a wall or obstacle cell leaves the pose unchanged and pays `illegal_move_reward`. Reaching the goal cell is terminal with `goal_reward`.
- **Discount** \(\gamma \in (0,1)\) (default \(0.85\)).

Action cost before the move (defaults in the viewer):

| Action            | Reward   |
| ----------------- | -------- |
| Straight          | \(-1\)   |
| Turn right / left | \(-5\)   |
| Turn around       | \(-10\)  |
| Illegal move      | \(-50\)  |
| Enter goal        | \(+100\) |

---

## Bellman backup

For a non-terminal state \(s\), the one-step action value using current estimates \(V\) is

\[
Q(s,a) = R(s,a) + \gamma \, V(s')
\]

where \((s', R, \text{done})\) comes from `simulate_step`, and \(V(s') = 0\) if `done`. The optimality backup is

\[
V\_{k+1}(s) = \max_a Q(s,a).
\]

Terminal goal states keep \(V = 0\) (reward is collected on the entering transition).

---

## Value iteration

Starting from \(V*0(s) = 0\) everywhere, repeat until the maximum change \(\max_s |V*{k+1}(s) - V_k(s)| < \theta\) (default \(\theta = 10^{-6}\)) or `max_iterations` is reached:

1. For each non-terminal state, compute \(V\_{k+1}(s)\) with the Bellman backup above.
2. **Gauss–Seidel** (default in the viewer): use updated \(V\_{k+1}\) immediately within the same sweep (`synchronous=False`).
3. **Jacobi**: compute all backups from a frozen copy \(V_k\), then replace (`synchronous=True`).

Both variants implement the same Bellman operator; Gauss–Seidel often converges in fewer sweeps.

**Policy:** after convergence, \(\pi(s) = \arg\max_a Q(s,a)\) (greedy with respect to the final \(V\)). The canvas shows per-cell policy arrows from the best action at each heading.

---

## Files

| File                  | Role                                         |
| --------------------- | -------------------------------------------- |
| `world.py`            | MDP: `simulate_step`, `bellman_action_value` |
| `value_iteration.py`  | VI sweeps (Jacobi / Gauss–Seidel)            |
| `grid_setup.py`       | Map editor                                   |
| `iteration_viewer.py` | Interactive GUI                              |
| `main.py`             | Entry loop                                   |
