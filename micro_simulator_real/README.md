# Policy runtime micro-simulator

Grelha única: setup → treino → dois painéis.

## Run

```bash
cd micro_simulator_real && python run.py
```

## Painéis

| Painel | Conteúdo |
|--------|----------|
| **Known map** (direita) | S, G, obstáculos, setas π para o **heading que escolheres** (N/E/S/W) — sem caminho verde, sem robot |
| **Execution** (esquerda) | Robot, rasto rosa, **caminho greedy verde** desde pose+heading **atual** (muda ao moveres manualmente) |

## Controlos

- **Train** — Q-learning no mapa  
- **Auto run** — start → goal (π automática)  
- **Next step** — um passo da política (virar **ou** avançar)  
- **Back to start**  
- **Manual** — N/E/S/W ou clique numa célula adjacente no mapa da esquerda  
