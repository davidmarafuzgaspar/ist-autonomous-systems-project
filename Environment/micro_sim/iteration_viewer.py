"""Interactive Tkinter viewer that steps through value iteration.

Click "Next iteration" to apply one Bellman sweep across the grid.
Each cell shows the current ``V(s)`` and a colored background so you
can see the goal value propagating outward in real time. A green
arrow shows the greedy action implied by the current ``V`` table.
"""

from __future__ import annotations

import time
import tkinter as tk

from .value_iteration import ValueIteration
from .viewer import _oriented_turn_arc_world_xy
from .world import (
    Action,
    GridCell,
    Heading,
    IntersectionWorld,
    MdpAction,
    MdpState,
    OrientedAction,
    PoseState,
)


LINE_WIDTH_M = 0.07
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.10

ARROW_COLOR = "#1976d2"
ARROW_OUTLINE = "#0d47a1"
START_OUTLINE = "#2f8f46"
GOAL_FILL = "#1565c0"
GOAL_OUTLINE = "#0d47a1"
OBSTACLE_FILL = "#c96c28"
OBSTACLE_OUTLINE = "#7a3d14"

POS_SCALE_MAX = 10000.0
NEG_SCALE_MAX = 500.0


_ACTION_UNIT_VEC: dict[Action, tuple[float, float]] = {
    Action.UP: (0.0, 1.0),
    Action.DOWN: (0.0, -1.0),
    Action.LEFT: (-1.0, 0.0),
    Action.RIGHT: (1.0, 0.0),
}

_HEADING_UNIT_VEC: dict[Heading, tuple[float, float]] = {
    Heading.N: (0.0, 1.0),
    Heading.E: (1.0, 0.0),
    Heading.S: (0.0, -1.0),
    Heading.W: (-1.0, 0.0),
}


class InteractiveValueIterationViewer:
    def __init__(
        self,
        world: IntersectionWorld,
        gamma: float = 0.85,
        theta: float = 1e-3,
        max_iterations: int = 1000,
        canvas_size_px: int = 700,
        sidebar_width_px: int = 320,
        synchronous: bool = False,
    ) -> None:
        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.canvas_size_px = canvas_size_px
        self.sidebar_width_px = sidebar_width_px
        self.synchronous = synchronous

        self.solver = ValueIteration(world, gamma=gamma, theta=theta, max_iterations=max_iterations, synchronous=synchronous)
        self.values: dict[MdpState, float] = self.solver.initial_values()
        self.policy: dict[MdpState, MdpAction | None] = self.solver.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta: float = float("inf")
        self.converged = False

        self.window = tk.Tk()
        self.window.title("Micro Sim - Interactive Value Iteration")
        self.window.configure(bg="#1c1c1c")

        self.canvas = tk.Canvas(
            self.window,
            width=canvas_size_px,
            height=canvas_size_px,
            bg="#1c1c1c",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, padx=8, pady=8)

        self.sidebar = tk.Frame(self.window, bg="#1c1c1c", width=sidebar_width_px)
        self.sidebar.grid(row=0, column=1, padx=8, pady=8, sticky="n")

        self.param_entries: dict[str, tk.Entry] = {}
        self.param_message: tk.Label | None = None
        self.algorithm_var = tk.StringVar(value="jacobi" if synchronous else "gauss")

        self._build_sidebar()
        self._draw()

    def run(self) -> None:
        self.window.mainloop()

    def _build_sidebar(self) -> None:
        title = tk.Label(
            self.sidebar,
            text="Value Iteration",
            font=("Arial", 16, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        )
        title.pack(anchor="w", pady=(4, 10))

        self.iter_label = tk.Label(self.sidebar, font=("Courier New", 12), fg="#eeeeee", bg="#1c1c1c", anchor="w", justify="left")
        self.iter_label.pack(anchor="w", pady=(0, 4))

        self.delta_label = tk.Label(self.sidebar, font=("Courier New", 12), fg="#eeeeee", bg="#1c1c1c", anchor="w", justify="left")
        self.delta_label.pack(anchor="w", pady=(0, 4))

        self.time_label = tk.Label(self.sidebar, text="time     : --", font=("Courier New", 12), fg="#90caf9", bg="#1c1c1c", anchor="w", justify="left")
        self.time_label.pack(anchor="w", pady=(0, 4))

        self.status_label = tk.Label(self.sidebar, font=("Courier New", 12, "bold"), fg="#ffd54f", bg="#1c1c1c", anchor="w", justify="left")
        self.status_label.pack(anchor="w", pady=(0, 12))

        button_style = {"font": ("Arial", 11, "bold"), "width": 22, "padx": 6, "pady": 4}

        self.step_btn = tk.Button(self.sidebar, text="Next iteration", command=self._on_step, bg="#388e3c", fg="white", activebackground="#2e7d32", **button_style)
        self.step_btn.pack(pady=4)

        self.run_btn = tk.Button(self.sidebar, text="Run to convergence", command=self._on_run_to_convergence, bg="#1976d2", fg="white", activebackground="#1565c0", **button_style)
        self.run_btn.pack(pady=4)

        self.reset_btn = tk.Button(self.sidebar, text="Reset (V = 0)", command=self._on_reset, bg="#d32f2f", fg="white", activebackground="#b71c1c", **button_style)
        self.reset_btn.pack(pady=4)

        if self.world.oriented_mdp:
            tk.Label(
                self.sidebar,
                text=(
                    "Modo orientado: por célula mostra argmax_a max_h Q (no start só h inicial).\n"
                    "Seta = F; arco = L/R. O desenho usa argmax_h Q para essa ação (sem letras N/E/S/W)."
                ),
                font=("Courier New", 8),
                fg="#9e9e9e",
                bg="#1c1c1c",
                anchor="w",
                justify="left",
                wraplength=self.sidebar_width_px - 8,
            ).pack(anchor="w", pady=(6, 0))

        self._build_parameter_panel()

    def _build_parameter_panel(self) -> None:
        separator = tk.Frame(self.sidebar, bg="#444", height=1)
        separator.pack(fill="x", pady=(14, 10))

        algo_title = tk.Label(
            self.sidebar,
            text="Algorithm",
            font=("Arial", 12, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        )
        algo_title.pack(anchor="w", pady=(0, 4))

        radio_style = {
            "bg": "#1c1c1c",
            "fg": "#cccccc",
            "selectcolor": "#2b2b2b",
            "activebackground": "#1c1c1c",
            "activeforeground": "#ffffff",
            "font": ("Courier New", 10),
            "anchor": "w",
            "highlightthickness": 0,
        }

        tk.Radiobutton(
            self.sidebar,
            text="Gauss-Seidel (in-place)",
            variable=self.algorithm_var,
            value="gauss",
            command=self._on_algorithm_change,
            **radio_style,
        ).pack(anchor="w")

        tk.Radiobutton(
            self.sidebar,
            text="Jacobi (synchronous)",
            variable=self.algorithm_var,
            value="jacobi",
            command=self._on_algorithm_change,
            **radio_style,
        ).pack(anchor="w", pady=(0, 8))

        separator2 = tk.Frame(self.sidebar, bg="#444", height=1)
        separator2.pack(fill="x", pady=(0, 10))

        title = tk.Label(
            self.sidebar,
            text="Parameters",
            font=("Arial", 12, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        )
        title.pack(anchor="w", pady=(0, 6))

        grid = tk.Frame(self.sidebar, bg="#1c1c1c")
        grid.pack(anchor="w")

        param_specs: list[tuple[str, str, float]] = [
            ("gamma", "gamma", self.gamma),
            ("goal_reward", "goal reward", self.world.goal_reward),
            ("collision_penalty", "collision", self.world.collision_penalty),
            ("away_from_goal_penalty", "away penalty", self.world.away_from_goal_penalty),
            ("step_cost", "step cost", self.world.step_cost),
            ("turn_90_reward", "turn 90°", self.world.turn_90_reward),
        ]

        for row, (key, label, value) in enumerate(param_specs):
            tk.Label(
                grid,
                text=label,
                font=("Courier New", 10),
                fg="#cccccc",
                bg="#1c1c1c",
                anchor="w",
                width=14,
            ).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
            entry = tk.Entry(
                grid,
                font=("Courier New", 10),
                width=10,
                bg="#2b2b2b",
                fg="#ffffff",
                insertbackground="#ffffff",
                relief="flat",
                highlightthickness=1,
                highlightbackground="#555",
                highlightcolor="#1976d2",
            )
            entry.insert(0, self._format_param(value))
            entry.grid(row=row, column=1, sticky="w", pady=2)
            self.param_entries[key] = entry

        apply_btn = tk.Button(
            self.sidebar,
            text="Apply changes",
            command=self._on_apply_parameters,
            bg="#7e57c2",
            fg="white",
            activebackground="#5e35b1",
            font=("Arial", 11, "bold"),
            width=22,
            padx=6,
            pady=4,
        )
        apply_btn.pack(pady=(10, 4))

        self.param_message = tk.Label(
            self.sidebar,
            text="",
            font=("Courier New", 9),
            fg="#ef9a9a",
            bg="#1c1c1c",
            anchor="w",
            justify="left",
            wraplength=self.sidebar_width_px - 16,
        )
        self.param_message.pack(anchor="w", pady=(0, 4))

    @staticmethod
    def _format_param(value: float) -> str:
        if float(value).is_integer():
            return str(int(value))
        return f"{value:g}"

    def _cell_value_map(self) -> dict[GridCell, float]:
        if self.world.oriented_mdp:
            return self.world.aggregate_max_v_per_cell(self.values)
        return self.values  # type: ignore[return-value]

    def _cell_policy_and_heading(
        self,
    ) -> tuple[dict[GridCell, MdpAction | None], dict[GridCell, Heading]]:
        if self.world.oriented_mdp:
            p = self.world.aggregated_policy_per_cell(self.values, self.gamma)
            h = self.world.display_heading_map_for_cell_policy(p, self.values, self.gamma)
            return p, h
        return self.policy, {}  # type: ignore[return-value]

    def _on_algorithm_change(self) -> None:
        self.synchronous = self.algorithm_var.get() == "jacobi"
        self.solver = ValueIteration(
            self.world,
            gamma=self.gamma,
            theta=self.theta,
            max_iterations=self.max_iterations,
            synchronous=self.synchronous,
        )
        self._on_reset()
        if self.param_message is not None:
            label = "Jacobi (synchronous)" if self.synchronous else "Gauss-Seidel (in-place)"
            self.param_message.config(text=f"switched to {label}", fg="#a5d6a7")

    def _on_apply_parameters(self) -> None:
        try:
            new_gamma = float(self.param_entries["gamma"].get())
            new_goal = float(self.param_entries["goal_reward"].get())
            new_collision = float(self.param_entries["collision_penalty"].get())
            new_away = float(self.param_entries["away_from_goal_penalty"].get())
            new_step = float(self.param_entries["step_cost"].get())
            new_t90 = float(self.param_entries["turn_90_reward"].get())
        except ValueError as exc:
            if self.param_message is not None:
                self.param_message.config(text=f"invalid number: {exc}", fg="#ef9a9a")
            return

        if not 0.0 < new_gamma <= 1.0:
            if self.param_message is not None:
                self.param_message.config(text="gamma must be in (0, 1]", fg="#ef9a9a")
            return

        self.world.goal_reward = new_goal
        self.world.collision_penalty = new_collision
        self.world.away_from_goal_penalty = new_away
        self.world.step_cost = new_step
        self.world.turn_90_reward = new_t90

        self.gamma = new_gamma
        self.solver = ValueIteration(
            self.world,
            gamma=self.gamma,
            theta=self.theta,
            max_iterations=self.max_iterations,
            synchronous=self.synchronous,
        )

        self._on_reset()

        if self.param_message is not None:
            self.param_message.config(
                text=(
                    f"applied. gamma={new_gamma}, goal={new_goal}, "
                    f"coll={new_collision}, away={new_away}, step={new_step}, "
                    f"t90={new_t90}"
                ),
                fg="#a5d6a7",
            )

    @property
    def _half_extent_m(self) -> float:
        max_axis = max(self.world.rows, self.world.cols)
        return (max_axis - 1) * self.world.spacing_m / 2.0 + self.world.spacing_m / 2.0 + OUTER_EXTENSION_M

    @property
    def _view_extent_m(self) -> float:
        return self._half_extent_m + MARGIN_M

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        extent = self._view_extent_m
        scale = self.canvas_size_px / (2.0 * extent)
        return (x_m + extent) * scale, (extent - y_m) * scale

    def _meters_to_pixels(self, meters: float) -> float:
        return meters * self.canvas_size_px / (2.0 * self._view_extent_m)

    def _line_centers(self) -> list[float]:
        n = self.world.cols
        half_span = (n - 1) / 2.0
        return [(index - half_span) * self.world.spacing_m for index in range(n)]

    def _on_step(self) -> None:
        if self.converged or self.iteration >= self.max_iterations:
            return
        self.values, self.last_delta = self.solver.step(self.values)
        self.iteration += 1
        if self.last_delta < self.theta:
            self.converged = True
        self.policy = self.solver.greedy_policy(self.values)
        self._draw()

    def _on_run_to_convergence(self) -> None:
        if self.converged:
            return

        algo_values = dict(self.values)
        algo_iter = self.iteration
        start = time.perf_counter()
        while algo_iter < self.max_iterations:
            algo_values, delta = self.solver.step(algo_values)
            algo_iter += 1
            if delta < self.theta:
                break
        algo_time_ms = (time.perf_counter() - start) * 1000.0
        algo_iters_done = algo_iter - self.iteration

        while not self.converged and self.iteration < self.max_iterations:
            self._on_step()
            self.window.update_idletasks()

        self.time_label.config(
            text=f"time     : {algo_time_ms:.3f} ms  ({algo_iters_done} iters)"
        )

    def _on_reset(self) -> None:
        self.values = self.solver.initial_values()
        self.policy = self.solver.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta = float("inf")
        self.converged = False
        self.time_label.config(text="time     : --")
        self._draw()

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_value_overlays()
        self._draw_obstacles()
        self._draw_policy_arrows()
        self._draw_start_and_goal_markers()
        self._update_sidebar_labels()

    def _update_sidebar_labels(self) -> None:
        self.iter_label.config(text=f"iteration : {self.iteration}")
        delta_str = "inf" if self.last_delta == float("inf") else f"{self.last_delta:.4f}"
        self.delta_label.config(text=f"delta     : {delta_str}")
        if self.iteration == 0:
            self.status_label.config(text="ready (V = 0)", fg="#ffd54f")
        elif self.converged:
            self.status_label.config(text="CONVERGED", fg="#81c784")
        else:
            self.status_label.config(text="running...", fg="#ffd54f")

    def _draw_board(self) -> None:
        half = self._half_extent_m
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="#444", width=2)

        line_width_px = self._meters_to_pixels(LINE_WIDTH_M)
        line_extent = half
        for center in self._line_centers():
            x0, y0 = self._world_to_canvas(center, line_extent)
            x1, y1 = self._world_to_canvas(center, -line_extent)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)

            x0, y0 = self._world_to_canvas(-line_extent, center)
            x1, y1 = self._world_to_canvas(line_extent, center)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)

    def _draw_value_overlays(self) -> None:
        box_half_m = self.world.spacing_m * 0.42
        cell_vals = self._cell_value_map()
        for cell, value in cell_vals.items():
            if self.world.is_obstacle(cell):
                continue
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - box_half_m, y_m + box_half_m)
            x1, y1 = self._world_to_canvas(x_m + box_half_m, y_m - box_half_m)
            fill = self._value_to_color(value)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#555", width=1)

            text_color = "#000000" if self._is_light(fill) else "#ffffff"
            cx, cy = self._world_to_canvas(x_m, y_m)
            self.canvas.create_text(
                cx,
                cy - self._meters_to_pixels(0.02),
                text=self._format_value(value),
                fill=text_color,
                font=("Courier New", 11, "bold"),
            )

    def _value_to_color(self, value: float) -> str:
        if value > 0:
            t = min(1.0, value / POS_SCALE_MAX)
            r = int(255 - 165 * t)
            g = int(255 - 50 * t)
            b = int(255 - 200 * t)
            return f"#{r:02x}{g:02x}{b:02x}"
        if value < 0:
            t = min(1.0, abs(value) / NEG_SCALE_MAX)
            r = int(255 - 30 * t)
            g = int(255 - 180 * t)
            b = int(255 - 180 * t)
            return f"#{r:02x}{g:02x}{b:02x}"
        return "#ffffff"

    @staticmethod
    def _is_light(hex_color: str) -> bool:
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        luminance = 0.299 * r + 0.587 * g + 0.114 * b
        return luminance > 140.0

    @staticmethod
    def _format_value(value: float) -> str:
        if abs(value) >= 1000.0:
            return f"{value:.0f}"
        return f"{value:.1f}"

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self.world.obstacles:
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - half, y_m + half)
            x1, y1 = self._world_to_canvas(x_m + half, y_m - half)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=OBSTACLE_FILL, outline=OBSTACLE_OUTLINE, width=2)

    def _draw_policy_arrows(self) -> None:
        if self.iteration == 0:
            return
        arrow_length_m = self.world.spacing_m * 0.28
        arrow_width_px = max(2.5, self._meters_to_pixels(0.010))
        head_long = max(8.0, self._meters_to_pixels(0.020))
        head_short = max(10.0, self._meters_to_pixels(0.024))
        head_wide = max(5.0, self._meters_to_pixels(0.014))
        arrow_offset_m = self.world.spacing_m * 0.12

        pol, head_map = self._cell_policy_and_heading()
        for cell, action in pol.items():
            if action is None:
                continue
            if cell == self.world.goal:
                continue
            x_m, y_m = self.world.world_xy(cell)

            if isinstance(action, OrientedAction):
                heading = head_map.get(cell, Heading.N)
                if action == OrientedAction.FORWARD:
                    unit_x, unit_y = _HEADING_UNIT_VEC[heading]
                    start_px = self._world_to_canvas(
                        x_m - unit_x * arrow_length_m / 2.0,
                        y_m - unit_y * arrow_length_m / 2.0 - arrow_offset_m,
                    )
                    end_px = self._world_to_canvas(
                        x_m + unit_x * arrow_length_m / 2.0,
                        y_m + unit_y * arrow_length_m / 2.0 - arrow_offset_m,
                    )
                    self.canvas.create_line(
                        start_px[0], start_px[1], end_px[0], end_px[1],
                        fill=ARROW_COLOR,
                        width=arrow_width_px,
                        arrow=tk.LAST,
                        arrowshape=(head_long, head_short, head_wide),
                        capstyle=tk.ROUND,
                    )
                else:
                    turn_left = action == OrientedAction.TURN_LEFT
                    arc_xy = _oriented_turn_arc_world_xy(
                        x_m, y_m, heading, turn_left, self.world.spacing_m * 0.13,
                    )
                    flat: list[float] = []
                    for wx, wy in arc_xy:
                        px, py = self._world_to_canvas(wx, wy)
                        flat.extend([px, py])
                    self.canvas.create_line(
                        *flat,
                        fill=ARROW_COLOR,
                        width=arrow_width_px,
                        smooth=True,
                        arrow=tk.LAST,
                        arrowshape=(head_long, head_short, head_wide),
                    )
            else:
                unit_x, unit_y = _ACTION_UNIT_VEC[action]
                start_px = self._world_to_canvas(
                    x_m - unit_x * arrow_length_m / 2.0,
                    y_m - unit_y * arrow_length_m / 2.0 - arrow_offset_m,
                )
                end_px = self._world_to_canvas(
                    x_m + unit_x * arrow_length_m / 2.0,
                    y_m + unit_y * arrow_length_m / 2.0 - arrow_offset_m,
                )
                self.canvas.create_line(
                    start_px[0], start_px[1], end_px[0], end_px[1],
                    fill=ARROW_COLOR,
                    width=arrow_width_px,
                    arrow=tk.LAST,
                    arrowshape=(head_long, head_short, head_wide),
                    capstyle=tk.ROUND,
                )

    def _draw_start_and_goal_markers(self) -> None:
        radius_px = max(10.0, self._meters_to_pixels(0.055))

        start_x_m, start_y_m = self.world.world_xy(self.world.start)
        sx, sy = self._world_to_canvas(start_x_m, start_y_m)
        self.canvas.create_oval(
            sx - radius_px, sy - radius_px,
            sx + radius_px, sy + radius_px,
            fill="",
            outline=START_OUTLINE,
            width=4,
        )
        self.canvas.create_text(sx, sy + radius_px + 12, text="S", fill=START_OUTLINE, font=("Arial", 11, "bold"))

        if self.world.oriented_mdp:
            ux, uy = _HEADING_UNIT_VEC[self.world.start_heading]
            tick = self._meters_to_pixels(self.world.spacing_m * 0.22)
            self.canvas.create_line(
                sx, sy,
                sx + ux * tick,
                sy - uy * tick,
                fill=START_OUTLINE,
                width=4,
                capstyle=tk.ROUND,
            )

        goal_x_m, goal_y_m = self.world.world_xy(self.world.goal)
        gx, gy = self._world_to_canvas(goal_x_m, goal_y_m)
        self.canvas.create_oval(
            gx - radius_px, gy - radius_px,
            gx + radius_px, gy + radius_px,
            fill=GOAL_FILL,
            outline=GOAL_OUTLINE,
            width=3,
        )
        self.canvas.create_text(gx, gy, text="G", fill="white", font=("Arial", 13, "bold"))
