"""Viewer Tkinter: Fase A (pi0*) e Fase B (Q-learning apos mudar mapa)."""

from __future__ import annotations

import tkinter as tk

from mdp_algorithm.display import oriented_policy_glyph
from mdp_algorithm.iteration_viewer import (
    ARROW_COLOR,
    GOAL_FILL,
    GOAL_OUTLINE,
    LINE_WIDTH_M,
    MARGIN_M,
    OBSTACLE_FILL,
    OBSTACLE_OUTLINE,
    OBSTACLE_SIZE_M,
    OUTER_EXTENSION_M,
    START_OUTLINE,
    _HEADING_UNIT_VEC,
    _oriented_turn_arc_world_xy,
)
from mdp_algorithm.world import GridCell, Heading, IntersectionWorld, OrientedAction, PoseState

from .narration import INTRO, explain_phase_a_step, explain_phase_b_step
from .viewer_session import ViewerMode, ViewerSession

REF_PATH_COLOR = "#66bb6a"
ROBOT_COLOR = "#e91e63"
TRAIL_COLOR = "#ff80ab"
Q_ARROW = "#42a5f5"
OLD_OBSTACLE = "#888888"


class QLearningViewer:
    def __init__(
        self,
        session: ViewerSession | None = None,
        *,
        auto_delay_ms: int = 400,
        canvas_size: int = 680,
        sidebar_w: int = 360,
    ) -> None:
        self.session = session or ViewerSession()
        self.auto_delay_ms = auto_delay_ms
        self.auto_on = False

        self.win = tk.Tk()
        self.win.title("Q-learning — explicacao visual")
        self.win.configure(bg="#1c1c1c")

        self.canvas = tk.Canvas(
            self.win, width=canvas_size, height=canvas_size,
            bg="#1c1c1c", highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, padx=8, pady=8)

        self.sidebar = tk.Frame(self.win, bg="#1c1c1c", width=sidebar_w)
        self.sidebar.grid(row=0, column=1, sticky="n")

        self._build_sidebar()
        self._draw()

    def run(self) -> None:
        self.win.mainloop()

    def _build_sidebar(self) -> None:
        tk.Label(
            self.sidebar, text="Q-learning passo a passo",
            font=("Arial", 15, "bold"), fg="#fff", bg="#1c1c1c",
        ).pack(anchor="w", pady=(4, 6))

        tk.Label(
            self.sidebar, text=INTRO, font=("Arial", 9), fg="#b0bec5", bg="#1c1c1c",
            justify="left", wraplength=self.sidebar.winfo_reqwidth() or 340,
        ).pack(anchor="w")

        tk.Label(
            self.sidebar, text="Agora:", font=("Arial", 11, "bold"),
            fg="#ffd54f", bg="#1c1c1c",
        ).pack(anchor="w", pady=(10, 2))

        self.narration = tk.Label(
            self.sidebar, text="", font=("Arial", 10), fg="#fff", bg="#2d2d2d",
            justify="left", wraplength=340, padx=8, pady=8,
        )
        self.narration.pack(anchor="w", fill="x", pady=4)

        self.status = tk.Label(
            self.sidebar, text="", font=("Courier New", 10), fg="#aaa", bg="#1c1c1c",
            justify="left", wraplength=340,
        )
        self.status.pack(anchor="w", pady=4)

        btn = {"font": ("Arial", 11, "bold"), "width": 22, "pady": 4}
        tk.Button(
            self.sidebar, text="Passo seguinte", command=self._on_step,
            bg="#388e3c", fg="white", **btn,
        ).pack(pady=3)
        tk.Button(
            self.sidebar, text="Mudar mapa (Fase B)", command=self._on_switch_b,
            bg="#f57c00", fg="white", **btn,
        ).pack(pady=3)
        tk.Button(
            self.sidebar, text="Novo episodio", command=self._on_new_ep,
            bg="#1976d2", fg="white", **btn,
        ).pack(pady=3)
        tk.Button(
            self.sidebar, text="Correr automatico", command=self._on_auto,
            bg="#7b1fa2", fg="white", **btn,
        ).pack(pady=3)
        tk.Button(
            self.sidebar, text="Pausa", command=self._on_pause,
            bg="#5d4037", fg="white", **btn,
        ).pack(pady=3)
        tk.Button(
            self.sidebar, text="Recomecar Fase A", command=self._on_reset_a,
            bg="#d32f2f", fg="white", **btn,
        ).pack(pady=3)

    def _on_step(self) -> None:
        self.session.step()
        self._draw()

    def _on_switch_b(self) -> None:
        self.auto_on = False
        self.session.switch_to_map_b()
        self._draw()

    def _on_new_ep(self) -> None:
        if self.session.mode == ViewerMode.FASE_B:
            self.session.begin_episode_b()
        else:
            self.session.begin_episode_a()
        self._draw()

    def _on_auto(self) -> None:
        self.auto_on = True
        self._tick()

    def _on_pause(self) -> None:
        self.auto_on = False

    def _on_reset_a(self) -> None:
        self.auto_on = False
        self.session = ViewerSession(gamma=self.session.gamma, max_steps=self.session.max_steps)
        self._draw()

    def _tick(self) -> None:
        if not self.auto_on:
            return
        if self.session.episode_done and self.session.mode == ViewerMode.FASE_A:
            self.session.switch_to_map_b()
        elif not self.session.episode_done:
            self.session.step()
        elif self.session.mode == ViewerMode.FASE_B:
            self.session.begin_episode_b()
        self._draw()
        self.win.after(self.auto_delay_ms, self._tick)

    @property
    def _world(self) -> IntersectionWorld:
        return self.session.display_world

    @property
    def _half(self) -> float:
        m = max(self._world.rows, self._world.cols)
        return (m - 1) * self._world.spacing_m / 2.0 + self._world.spacing_m / 2.0 + OUTER_EXTENSION_M

    @property
    def _extent(self) -> float:
        return self._half + MARGIN_M

    def _w2c(self, x: float, y: float) -> tuple[float, float]:
        e = self._extent
        s = self.canvas.winfo_width() or 680
        s = max(s, 100)
        return (x + e) * s / (2 * e), (e - y) * s / (2 * e)

    def _m2p(self, m: float) -> float:
        s = self.canvas.winfo_width() or 680
        return m * s / (2 * self._extent)

    def _line_centers(self) -> list[float]:
        h = (self._world.cols - 1) / 2.0
        return [(i - h) * self._world.spacing_m for i in range(self._world.cols)]

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_ref_path()
        self._draw_obstacles()
        if self.session.mode == ViewerMode.FASE_B:
            self._draw_q_arrows()
        else:
            self._draw_pi0_arrows()
        self._draw_trail()
        self._draw_robot()
        self._update_text()

    def _draw_board(self) -> None:
        h = self._half
        x0, y0 = self._w2c(-h, h)
        x1, y1 = self._w2c(h, -h)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="#444", width=2)
        lw = self._m2p(LINE_WIDTH_M)
        for c in self._line_centers():
            self.canvas.create_line(*self._w2c(c, h), *self._w2c(c, -h), fill="#333", width=lw)
            self.canvas.create_line(*self._w2c(-h, c), *self._w2c(h, c), fill="#333", width=lw)

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self._world.obstacles:
            x, y = self._world.world_xy(cell)
            x0, y0 = self._w2c(x - half, y + half)
            x1, y1 = self._w2c(x + half, y - half)
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill=OBSTACLE_FILL, outline=OBSTACLE_OUTLINE, width=2,
            )
        if self.session.mode == ViewerMode.FASE_B and self.session.world_o1:
            old_only = self.session.world_o0.obstacles - self._world.obstacles
            for cell in old_only:
                x, y = self._world.world_xy(cell)
                x0, y0 = self._w2c(x - half, y + half)
                x1, y1 = self._w2c(x + half, y - half)
                self.canvas.create_rectangle(x0, y0, x1, y1, outline=OLD_OBSTACLE, dash=(4, 3), width=2)

    def _draw_ref_path(self) -> None:
        if len(self.session.reference_path) < 2:
            return
        pts: list[float] = []
        for cell in self.session.reference_path:
            x, y = self._world.world_xy(cell)
            px, py = self._w2c(x, y)
            pts.extend([px, py])
        self.canvas.create_line(*pts, fill=REF_PATH_COLOR, width=3, dash=(6, 4))

    def _draw_pi0_arrows(self) -> None:
        if self.session.mode != ViewerMode.FASE_A:
            return
        for cell in self.session.reference_path:
            for h in Heading:
                pose = PoseState(cell, h)
                act = self.session.phase_a.policy.get(pose)
                if act:
                    self._arrow_at(cell, h, act, REF_PATH_COLOR, small=True)

    def _draw_q_arrows(self) -> None:
        seen: set[PoseState] = {s for s, _ in self.session.q._q}
        for s in seen:
            if self._world.is_terminal(s):
                continue
            act = self.session.q.greedy_action(s, self._world)
            self._arrow_at(s.cell, s.heading, act, Q_ARROW, small=True)

    def _arrow_at(self, cell: GridCell, heading: Heading, action: OrientedAction, color: str, small: bool) -> None:
        x, y = self._world.world_xy(cell)
        ln = self._world.spacing_m * (0.16 if small else 0.24)
        aw = max(1.5, self._m2p(0.006))
        if action == OrientedAction.FORWARD:
            ux, uy = _HEADING_UNIT_VEC[heading]
            self.canvas.create_line(
                *self._w2c(x - ux * ln / 2, y - uy * ln / 2),
                *self._w2c(x + ux * ln / 2, y + uy * ln / 2),
                fill=color, width=aw, arrow=tk.LAST,
            )
        else:
            arc = _oriented_turn_arc_world_xy(
                x, y, heading, action == OrientedAction.TURN_LEFT, self._world.spacing_m * 0.08,
            )
            flat: list[float] = []
            for wx, wy in arc:
                flat.extend(self._w2c(wx, wy))
            self.canvas.create_line(*flat, fill=color, width=aw, smooth=True)

    def _draw_trail(self) -> None:
        if len(self.session.trail) < 2:
            return
        pts: list[float] = []
        for st in self.session.trail:
            x, y = self._world.world_xy(st.cell)
            pts.extend(self._w2c(x, y))
        self.canvas.create_line(*pts, fill=TRAIL_COLOR, width=2)

    def _draw_robot(self) -> None:
        if self.session.state is None:
            return
        st = self.session.state
        x, y = self._world.world_xy(st.cell)
        cx, cy = self._w2c(x, y)
        r = max(10, self._m2p(0.05))
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=ROBOT_COLOR, outline="white", width=2)
        ux, uy = _HEADING_UNIT_VEC[st.heading]
        t = self._m2p(self._world.spacing_m * 0.2)
        self.canvas.create_line(cx, cy, cx + ux * t, cy - uy * t, fill="white", width=3)

        gx, gy = self._w2c(*self._world.world_xy(self._world.goal))
        self.canvas.create_oval(gx - r, gy - r, gx + r, gy + r, fill=GOAL_FILL, outline=GOAL_OUTLINE, width=2)
        self.canvas.create_text(gx, gy, text="G", fill="white", font=("Arial", 11, "bold"))

    def _update_text(self) -> None:
        s = self.session
        phase = "FASE A (mapa antigo, plano otimo pi0*)" if s.mode == ViewerMode.FASE_A else "FASE B (mapa novo, Q-learning)"
        self.status.config(
            text=(
                f"{phase}\n"
                f"Episodio {s.episode}  |  passo {s.step_in_episode}  |  "
                f"R={s.episode_return:.0f}  |  colisoes={s.collisions_ep}"
            ),
        )
        if s.last is None:
            if s.mode == ViewerMode.FASE_A:
                self.narration.config(
                    text="Carrega 'Passo seguinte' para ver o caminho otimo pi0* no mapa antigo.\n"
                    "Linha verde = esse caminho.",
                )
            else:
                self.narration.config(
                    text="FASE B: o plano verde ja nao funciona.\n"
                    "Carrega Passo — primeiro segue pi0*; apos colisao aprende Q.",
                )
            return

        ls = s.last
        if s.mode == ViewerMode.FASE_A:
            self.narration.config(
                text=explain_phase_a_step(s.step_in_episode, ls.action, ls.reward, ls.hit_wall),
            )
        else:
            assert s.adapter is not None
            self.narration.config(
                text=explain_phase_b_step(
                    s.step_in_episode, ls.action, ls.reward, ls.hit_wall,
                    s.adapter, s.q, s.state or s.world_o1.initial_state(),
                    ls.old_q, ls.new_q,
                ),
            )
