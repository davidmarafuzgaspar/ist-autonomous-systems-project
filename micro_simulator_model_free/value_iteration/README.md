# Value iteration

Discrete MDP on an intersection grid with orientation. Map setup → interactive Bellman sweeps → optional terminal rollout.

## Model

**State** \(s = (r, c, h)\): cell \((r,c)\), heading \(h \in \{N,E,S,W\}\) (row 0 is north).

**Actions** \(a \in \{\text{straight}, \text{turn R}, \text{turn L}, \text{turn A}\}\):

1. Apply the turn (if any) to obtain \(h'\).
2. Attempt one step along \(h'\) to \((r', c')\).

**Transitions** (deterministic):

- If \((r',c')\) is blocked or out of bounds: stay at \((r,c,h)\), reward \(r_{\text{illegal}}\).
- If \((r',c') =\) goal: move to \((r',c',h')\), reward \(r_{\text{goal}}\), episode ends.
- Otherwise: move to \((r',c',h')\), reward \(R(a)\) (step / turn cost).

**Terminal states:** all poses at the goal cell; \(V(\text{goal}) = 0\).

## Rewards (defaults)

| Symbol | Default | Meaning |
|--------|---------|---------|
| \(\gamma\) | 0.85 | Discount |
| \(r_{\text{goal}}\) | +100 | Enter goal |
| \(r_{\text{illegal}}\) | −50 | Blocked forward step |
| \(R(\text{straight})\) | −1 | Forward move |
| \(R(\text{turn R/L})\) | −5 | Quarter turn |
| \(R(\text{turn A})\) | −10 | Half turn |

Editable in the VI viewer (**Apply rewards**).

## Bellman optimality

\[
V^*(s) = \max_a \Big[ R(s,a) + \gamma \sum_{s'} P(s'|s,a)\, V^*(s') \Big]
\]

With deterministic \(s' = f(s,a)\):

\[
Q(s,a) = R(s,a) + \gamma\, V^*\big(f(s,a)\big), \qquad
\pi^*(s) = \arg\max_a Q(s,a).
\]

One backup for non-terminal \(s\):

\[
V_{k+1}(s) \leftarrow \max_a Q_k(s,a), \quad Q_k(s,a) = R(s,a) + \gamma\, V_k(f(s,a)).
\]

**Stop** when \(\max_s |V_{k+1}(s) - V_k(s)| < \theta\) (default \(\theta = 10^{-6}\)) or after 200 iterations.

## Update rules (viewer)

| Mode | Sweep |
|------|--------|
| **Gauss–Seidel** | Uses \(V_{k+1}\) as soon as each state is updated (in-place). |
| **Jacobi** | Computes all backups from a frozen \(V_k\), then commits. |

Both converge to the same \(V^*\) and \(\pi^*\); intermediate values and iteration counts differ.

## Usage

```bash
cd micro_simulator_model_free
python -m value_iteration.main
```

| Flag | Effect |
|------|--------|
| `--final` | Terminal: solve, print V\*/π\*, rollout |
| `--skip-setup` | Skip map editor |
| `--rows`, `--cols` | Grid size when setup is skipped (default 3×5) |

**Flow:** Map setup → VI viewer → **Change world** (reopens setup with the last map).

Grid axes: 2–12.
