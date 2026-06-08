# Policy runtime micro-simulator

Two maps: **known** (VI training + π panel) and **true** (execution + hidden). Forward sensing on move → replan popup if the plan is blocked.

## Run

```bash
cd micro_simulator_dynamic && python run.py
```

## Setup

| Mode | Effect |
|------|--------|
| **Obstacle** | Known for training and in the real world |
| **Hidden** | Real world only until the robot moves forward (automatic sense) |
| Start / Goal / heading | Same as other micro-sims |

## Panels

| Panel | Content |
|-------|---------|
| **Known map** | Empty until Train; then S, G, known obstacles, π arrows (heading N/E/S/W) — no green path |
| **Execution** | True world, robot, trail, green greedy path (current known map) |

## Hidden + replan flow

1. **Train (Q-learning on known map)** — tabular Q-learning on the known map (`MODE_DYNAMIC`).
2. On forward move, the robot senses the cell ahead.
3. New hidden cell → becomes a known obstacle.
4. If the current greedy plan used that cell → **popup**: replan with VI from the current pose?
5. **Yes** → replan; **No** → keep the previous policy.

## Controls

Train · Auto run · Back to start · Next step · manual (left map). Hidden = orange on setup + execution only.
