# Q-learning micro-simulator

Visualização do **model-free do `solver.py`**: o mesmo Q-learning tabular que `Solver._train_model_free` (ε-greedy, atualização TD, ε decai por episódio). Não é o robot ROS em tempo real — é o mesmo algoritmo num grelha editável para veres o que acontece.

## O que estás a ver

1. **Map setup** — defines grelha, início, objetivo (como escolher o mapa antes de treinar).
2. **Janela principal** — cada **episódio** o agente começa no verde, tenta chegar ao azul:
   - **Next step** — um passo do loop do solver (escolher ação → simular → atualizar Q).
   - **Run episode** — um episódio completo animado (trail rosa).
   - **Train all episodes** — corre os episódios restantes de uma vez (como `solver.train()` no terminal).
   - **Show learned policy** — mapa de ações gananciosas (como `format_policy_report()` no log).
3. **Train all episodes** até `Episodes: 1000/1000 — training complete`. Só então a grelha mostra **max Q + setas** (para onde ir) e podes escolher **N/E/S/W**; linha verde = plano desde o start.
4. **Show policy (text)** — mapa ASCII com **os 4 headings** (só disponível quando o treino terminou).

No `main.py` do robot, o solver treina **sem janela** e só imprime estatísticas; aqui vês o mesmo processo passo a passo ou em lote.

## Run

Requires **numpy** and **tkinter**.

**Inside this folder** (your usual case):

```bash
python run.py
```

**From the repository root**:

```bash
python -m micro_simulator_model_free
```

`python -m micro_simulator_model_free` only works when the **parent** of this directory is on `sys.path` (i.e. you are at the repo root, not inside `micro_simulator_model_free/`).

## Flow

1. **Map setup** — lines × columns, start/goal/heading, optional obstacles.
2. **Viewer** — step through learning, run full episodes, reset Q, edit α / γ / ε / rewards.
3. **Change world** — return to setup (previous map pre-filled).

## Algorithm (short)

- State: `(cell, heading)`; Q-table shape `(rows, cols, 4, 4)`.
- Update: `Q ← Q + α (r + γ max_a' Q(s',a') − Q)` (terminal: target = `r`).
- Exploration: ε-greedy; ε multiplied by `epsilon_decay` after each episode until `epsilon_end`.

Defaults: `α=0.2`, `γ=0.85`, `ε: 1.0 → 0.05` (decay `0.995`), `1000` episodes, `50` steps/episode.
