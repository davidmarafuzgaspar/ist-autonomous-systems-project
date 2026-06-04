# MDP não determinístico

Cópia do simulador `mdp_algorithm` com **movimento à frente estocástico**: ao executar **F**, o robô pode deslizar perpendicularmente à direção pretendida.

## Probabilidades (configuráveis)

Por defeito (renormalizadas se a soma ≠ 1):

| Resultado                          | Peso |
| ---------------------------------- | ---- |
| Avançar na direção do heading      | 70%  |
| Deslize à esquerda (perpendicular) | 15%  |
| Deslize à direita (perpendicular)  | 15%  |

**L** e **R** continuam determinísticos.

A Value Iteration usa a **expectativa** de Bellman:

\[
Q(s,F) = \sum\_{s'} P(s'|s,F)\,\bigl[R + \gamma V(s')\bigr]
\]

## Como correr

```bash
cd Environment_non_deterministic
python -m mdp_algorithm.main              # viewer interativo
python -m mdp_algorithm.main --final      # VI + rollouts no terminal
python -m mdp_algorithm.main --p-forward 0.8 --p-left 0.1 --p-right 0.1
```

No viewer: painel lateral → **P fwd / P slip left / P slip right** → **Apply**.

## Ficheiros

- `mdp_algorithm/world.py` — `transition_distribution`, `bellman_action_value`, `sample_transition`
- `mdp_algorithm/value_iteration.py` — VI com Bellman estocástico
- `mdp_algorithm/iteration_viewer.py` — parâmetros de slip no painel

O módulo determinístico de referência está em `Environment/mdp_algorithm/`.
