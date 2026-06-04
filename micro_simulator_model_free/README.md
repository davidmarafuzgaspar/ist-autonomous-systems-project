# Q-learning micro-simulator

Visualização do **model-free do `solver.py`**: Q-learning tabular (ε-greedy, TD, ε decai por episódio). Não é o robot ROS — é o mesmo algoritmo numa grelha editável.

## Run

Requires **numpy** and **tkinter**.

```bash
cd micro_simulator_model_free && python run.py
# or from repo root:
python -m micro_simulator_model_free
```

Dentro da pasta usa `run.py`; `-m` só a partir da raiz do repositório.

## Fluxo

1. **Map setup** — grelha **5×5** por defeito; start, goal, heading, obstáculos.
2. **Viewer** — Next step / Run episode / Train all; política com setas ↑→↓← só no fim do treino.
3. **Change world** — volta ao setup (mapa anterior pré-preenchido).

## Módulos

| Ficheiro | Conteúdo |
|----------|----------|
| `model.py` | Grelha orientada (`IntersectionWorld`) + `QLearningTrainer` |
| `gui.py` | Editor de mapa + `QLearningViewer` |
| `main.py` | Loop setup → viewer |
| `run.py` | Launcher |

Defaults: `α=0.2`, `γ=0.85`, `ε: 1.0 → 0.05` (decay `0.995`), `1000` episódios, `50` passos/episódio.
