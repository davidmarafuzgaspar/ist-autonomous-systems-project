"""Textos para o viewer (portugues)."""

from __future__ import annotations

from mdp_algorithm.world import OrientedAction, PoseState

from .experiment import OnlineAdapter
from .agent import QTable

_ACTION = {
    OrientedAction.FORWARD: "Frente",
    OrientedAction.TURN_LEFT: "Esquerda",
    OrientedAction.TURN_RIGHT: "Direita",
}


INTRO = """Q-learning em 2 fases:

FASE A (mapa antigo O0)
  O computador calcula pi0* (MDP).
  Carrega Passo para ver o caminho otimo.

FASE B (mapa novo O1)
  Obstaculos mudaram. O plano pi0* ja nao serve.
  Apos colisao, a formula Q aprende:

  Q <- Q + alpha * (r + gamma*max Q' - Q)

Robo rosa = tu. Setas verdes = caminho
otimo antigo. Setas azuis = o que Q recomenda."""


def explain_phase_a_step(step: int, action: OrientedAction, reward: float, hit_wall: bool) -> str:
    lines = [
        f"FASE A — passo {step}",
        f"Seguimos pi0* (plano otimo no mapa ANTIGO).",
        f"Acao: {_ACTION[action]}.",
    ]
    if hit_wall:
        lines.append("Isto nao devia acontecer em O0.")
    elif reward > 5000:
        lines.append("Goal! Fim do caminho de referencia.")
    else:
        lines.append(f"Recompensa {reward:.0f}. (Ainda nao e Q-learning.)")
    lines.append("\nQuando terminares, carrega 'Mudar mapa'.")
    return "\n".join(lines)


def explain_phase_b_step(
    step: int,
    action: OrientedAction,
    reward: float,
    hit_wall: bool,
    adapter: OnlineAdapter,
    q: QTable,
    state: PoseState,
    old_q: float,
    new_q: float,
) -> str:
    if adapter.use_old_policy_until_collision and not adapter.had_collision:
        mode = "Segue pi0* (plano do mapa antigo)"
    elif not adapter.had_collision:
        mode = "Escolha mista (pi0* ou exploracao em Q)"
    else:
        mode = "Usa Q-learning (plano antigo falhou)"

    lines = [
        f"FASE B — passo {step}  |  {mode}",
        f"Acao: {_ACTION[action]}.  r = {reward:.0f}",
    ]
    if hit_wall:
        lines.append("COLISAO! O obstaculo mudou de sitio.")
        lines.append("Q-learning baixa o valor desta acao:")
    lines.append(f"  Q(s,a): {old_q:.0f} -> {new_q:.0f}")
    lines.append(f"  (alpha mistura recompensa + futuro)")
    if reward > 5000:
        lines.append("Chegaste ao GOAL com o Q aprendido!")
    return "\n".join(lines)
