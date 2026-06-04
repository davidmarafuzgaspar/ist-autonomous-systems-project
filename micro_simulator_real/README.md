# Policy runtime micro-simulator

Grelha **única** (como model-free): defines obstáculos no setup, treinas, vês a **política ótima** no mapa e executas o robot.

Sem (por agora): obstáculos escondidos, sense, replan.

## Run

```bash
cd micro_simulator_real && python run.py
```

## Fluxo

1. **Map setup** — rows/cols, Obstacle / Start / Goal, heading inicial → Continue  
2. **Train** — Q-learning (solver) no mapa  
3. **Mapa + π** — setas azuis para o heading escolhido (N/E/S/W; **↻ robot** = heading atual do robot)  
4. **Execute** — Auto run, Back to start, Policy turn, Move forward  
5. **Manual** — botões N/E/S/W ou clique numa célula **adjacente**

Rasto cor-de-rosa no auto-run e nos movimentos. Contorno verde = caminho greedy desde o robot.

## Módulos

| Ficheiro | Role |
|----------|------|
| `model.py` | `Scenario`, `RealRuntimeSim` |
| `gui.py` | Setup + viewer |
| `main.py` | Loop |
