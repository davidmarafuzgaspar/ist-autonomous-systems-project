# Policy runtime micro-simulator

Dois mapas: **known** (treino VI + painel π) e **true** (execução + hidden). Sense à frente → popup de replan se bloquear o plano.

## Run

```bash
cd micro_simulator_real && python run.py
```

## Setup

| Modo | Efeito |
|------|--------|
| **Obstacle** | Conhecido no treino e na realidade |
| **Hidden** | Só na realidade até **Sense ahead** / avanço |
| Start / Goal / heading | Como nos outros sims |

## Painéis

| Painel | Conteúdo |
|--------|----------|
| **Known map** | S, G, obstáculos conhecidos, setas π (heading N/E/S/W) — sem caminho verde |
| **Execution** | Mundo real, robot, rasto, caminho verde (greedy no mapa conhecido atual) |

## Fluxo hidden + replan

1. **Train (VI on known map)** — value iteration no mapa conhecido (`MODE_MODEL_BASED`).
2. Ao avançar ou **Sense ahead**, vê a célula à frente.
3. Se for hidden novo → passa a obstáculo conhecido.
4. Se o plano greedy atual passava por essa célula → **popup**: replan com VI a partir da pose atual?
5. **Sim** → replan; **Não** → mantém π antiga.

## Controlos

Train · Auto run · Back to start · Sense ahead · Next step · manual (mapa esquerda) · Show hidden (debug)
