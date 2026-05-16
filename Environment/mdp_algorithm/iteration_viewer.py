"""Interactive Tkinter viewer: step through Bellman value iteration."""

from __future__ import annotations

import math
import time
import tkinter as tk

from .value_iteration import ValueIteration
from .world import GridCell, Heading, IntersectionWorld, OrientedAction, PoseState

LINE_WIDTH_M = 0.07
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.10

ARROW_COLOR = "#1976d2"
START_OUTLINE = "#2f8f46"
GOAL_FILL = "#1565c0"
GOAL_OUTLINE = "#0d47a1"
OBSTACLE_FILL = "#c96c28"
OBSTACLE_OUTLINE = "#7a3d14"

POS_SCALE_MAX = 10000.0
NEG_SCALE_MAX = 500.0

_HEADING_UNIT_VEC: dict[Heading, tuple[float, float]] = {
    Heading.N: (0.0, 1.0),
    Heading.E: (1.0, 0.0),
    Heading.S: (0.0, -1.0),
    Heading.W: (-1.0, 0.0),
}


def _oriented_turn_arc_world_xy(
    x_m: float,
    y_m: float,
    heading: Heading,
    turn_left: bool,
    radius_m: float,
    segments: int = 14,
) -> list[tuple[float, float]]:
    u0x, u0y = _HEADING_UNIT_VEC[heading]
    h1 = heading.turn_left() if turn_left else heading.turn_right()
    u1x, u1y = _HEADING_UNIT_VEC[h1]
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        t = (i / segments) * (math.pi / 2)
        pts.append(
            (
                x_m + radius_m * (math.cos(t) * u0x + math.sin(t) * u1x),
                y_m + radius_m * (math.cos(t) * u0y + math.sin(t) * u1y),
            )
        )
    return pts


class InteractiveValueIterationViewer:
    def __init__(
        self,
        world: IntersectionWorld,
        gamma: float = 0.85,
        theta: float = 1e-3,
        max_iterations: int = 1000,
        synchronous: bool = False,
        canvas_size_px: int = 700,
        sidebar_width_px: int = 300,
    ) -> None:
        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.synchronous = synchronous
        self.canvas_size_px = canvas_size_px
        self.sidebar_width_px = sidebar_width_px

        self.param_entries: dict[str, tk.Entry] = {}
        self.param_message: tk.Label | None = None
        self.algorithm_var: tk.StringVar

        self.solver = self._make_solver()
        self.values: dict[PoseState, float] = self.solver.initial_values()
        self.policy: dict[PoseState, OrientedAction | None] = self.solver.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta = float("inf")
        self.converged = False

        self.window = tk.Tk()
        self.window.title("Micro Sim - Value Iteration")
        self.window.configure(bg="#1c1c1c")
        self.algorithm_var = tk.StringVar(
            master=self.window,
            value="jacobi" if synchronous else "gauss",
        )

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

        self._build_sidebar()
        self._draw()

    def _make_solver(self) -> ValueIteration:
        return ValueIteration(
            self.world,
            gamma=self.gamma,
            theta=self.theta,
            max_iterations=self.max_iterations,
            synchronous=self.synchronous,
        )

    def run(self) -> None:
        self.window.mainloop()

    def _build_sidebar(self) -> None:
        tk.Label(
            self.sidebar,
            text="Value Iteration",
            font=("Arial", 16, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        ).pack(anchor="w", pady=(4, 10))

        self.iter_label = tk.Label(
            self.sidebar, font=("Courier New", 12), fg="#eeeeee", bg="#1c1c1c", anchor="w",
        )
        self.iter_label.pack(anchor="w", pady=(0, 4))

        self.delta_label = tk.Label(
            self.sidebar, font=("Courier New", 12), fg="#eeeeee", bg="#1c1c1c", anchor="w",
        )
        self.delta_label.pack(anchor="w", pady=(0, 4))

        self.time_label = tk.Label(
            self.sidebar,
            text="time     : --",
            font=("Courier New", 12),
            fg="#90caf9",
            bg="#1c1c1c",
            anchor="w",
        )
        self.time_label.pack(anchor="w", pady=(0, 4))

        self.status_label = tk.Label(
            self.sidebar, font=("Courier New", 12, "bold"), fg="#ffd54f", bg="#1c1c1c", anchor="w",
        )
        self.status_label.pack(anchor="w", pady=(0, 12))

        btn = {"font": ("Arial", 11, "bold"), "width": 20, "padx": 6, "pady": 4}
        tk.Button(
            self.sidebar, text="Next iteration", command=self._on_step,
            bg="#388e3c", fg="white", activebackground="#2e7d32", **btn,
        ).pack(pady=4)
        tk.Button(
            self.sidebar, text="Run to convergence", command=self._on_run_to_convergence,
            bg="#1976d2", fg="white", activebackground="#1565c0", **btn,
        ).pack(pady=4)
        tk.Button(
            self.sidebar, text="Reset (V = 0)", command=self._on_reset,
            bg="#d32f2f", fg="white", activebackground="#b71c1c", **btn,
        ).pack(pady=4)


        self._build_algorithm_panel()
        self._build_parameter_panel()

    def _build_algorithm_panel(self) -> None:
        tk.Frame(self.sidebar, bg="#444", height=1).pack(fill="x", pady=(14, 10))

        tk.Label(
            self.sidebar,
            text="Algorithm",
            font=("Arial", 12, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        ).pack(anchor="w", pady=(0, 4))

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
        ).pack(anchor="w", pady=(0, 4))

    def _build_parameter_panel(self) -> None:
        tk.Frame(self.sidebar, bg="#444", height=1).pack(fill="x", pady=(10, 10))

        tk.Label(
            self.sidebar,
            text="Rewards & gamma",
            font=("Arial", 12, "bold"),
            fg="#eeeeee",
            bg="#1c1c1c",
        ).pack(anchor="w", pady=(0, 6))

        grid = tk.Frame(self.sidebar, bg="#1c1c1c")
        grid.pack(anchor="w")

        specs: list[tuple[str, str, float]] = [
            ("gamma", "gamma", self.gamma),
            ("goal_reward", "goal reward", self.world.goal_reward),
            ("collision_penalty", "collision", self.world.collision_penalty),
            ("away_from_goal_penalty", "away penalty", self.world.away_from_goal_penalty),
            ("step_cost", "step cost", self.world.step_cost),
            ("turn_90_reward", "turn 90°", self.world.turn_90_reward),
        ]
        entry_style = {
            "font": ("Courier New", 10),
            "width": 10,
            "bg": "#2b2b2b",
            "fg": "#ffffff",
            "insertbackground": "#ffffff",
            "relief": "flat",
            "highlightthickness": 1,
            "highlightbackground": "#555",
            "highlightcolor": "#1976d2",
        }
        for row, (key, label, value) in enumerate(specs):
            tk.Label(
                grid,
                text=label,
                font=("Courier New", 10),
                fg="#cccccc",
                bg="#1c1c1c",
                anchor="w",
                width=14,
            ).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
            entry = tk.Entry(grid, **entry_style)
            entry.insert(0, self._format_param(value))
            entry.grid(row=row, column=1, sticky="w", pady=2)
            self.param_entries[key] = entry

        tk.Button(
            self.sidebar,
            text="Apply changes",
            command=self._on_apply_parameters,
            bg="#7e57c2",
            fg="white",
            activebackground="#5e35b1",
            font=("Arial", 11, "bold"),
            width=20,
            padx=6,
            pady=4,
        ).pack(pady=(10, 4))

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

    def _on_algorithm_change(self) -> None:
        self.synchronous = self.algorithm_var.get() == "jacobi"
        self.solver = self._make_solver()
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

        self.gamma = new_gamma
        self.world.goal_reward = new_goal
        self.world.collision_penalty = new_collision
        self.world.away_from_goal_penalty = new_away
        self.world.step_cost = new_step
        self.world.turn_90_reward = new_t90
        self.solver = self._make_solver()
        self._on_reset()

        if self.param_message is not None:
            self.param_message.config(
                text=(
                    f"applied: gamma={new_gamma:g}, goal={new_goal:g}, "
                    f"coll={new_collision:g}, away={new_away:g}, "
                    f"step={new_step:g}, turn={new_t90:g}"
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
        half_span = (self.world.cols - 1) / 2.0
        return [(index - half_span) * self.world.spacing_m for index in range(self.world.cols)]

    def _cell_policy_and_heading(
        self,
    ) -> tuple[dict[GridCell, OrientedAction | None], dict[GridCell, Heading]]:
        pol = self.world.aggregated_policy_per_cell(self.values, self.gamma)
        head = self.world.display_heading_map_for_cell_policy(pol, self.values, self.gamma)
        return pol, head

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
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        iters_done = algo_iter - self.iteration
        while not self.converged and self.iteration < self.max_iterations:
            self._on_step()
            self.window.update_idletasks()
        self.time_label.config(text=f"time     : {elapsed_ms:.1f} ms ({iters_done} iters)")

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
        self._draw_start_and_goal()
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
        for center in self._line_centers():
            x0, y0 = self._world_to_canvas(center, half)
            x1, y1 = self._world_to_canvas(center, -half)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)
            x0, y0 = self._world_to_canvas(-half, center)
            x1, y1 = self._world_to_canvas(half, center)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)

    def _draw_value_overlays(self) -> None:
        box_half_m = self.world.spacing_m * 0.42
        cell_vals = self.world.aggregate_max_v_per_cell(self.values)
        for cell, value in cell_vals.items():
            if self.world.is_obstacle(cell):
                continue
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - box_half_m, y_m + box_half_m)
            x1, y1 = self._world_to_canvas(x_m + box_half_m, y_m - box_half_m)
            fill = self._value_to_color(value)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#666", width=1)
            cx, cy = self._world_to_canvas(x_m, y_m)
            self.canvas.create_text(
                cx, cy,
                text=self._format_value(value),
                fill="#111111" if self._is_light(fill) else "#ffffff",
                font=("Courier New", 9, "bold"),
            )

    def _value_to_color(self, value: float) -> str:
        if value > 0:
            t = min(1.0, value / POS_SCALE_MAX)
            r, g, b = int(255 - 165 * t), int(255 - 50 * t), int(255 - 200 * t)
            return f"#{r:02x}{g:02x}{b:02x}"
        if value < 0:
            t = min(1.0, abs(value) / NEG_SCALE_MAX)
            r, g, b = int(255 - 30 * t), int(255 - 180 * t), int(255 - 180 * t)
            return f"#{r:02x}{g:02x}{b:02x}"
        return "#ffffff"

    @staticmethod
    def _is_light(hex_color: str) -> bool:
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return 0.299 * r + 0.587 * g + 0.114 * b > 140.0

    @staticmethod
    def _format_value(value: float) -> str:
        return f"{value:.0f}" if abs(value) >= 1000.0 else f"{value:.1f}"

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self.world.obstacles:
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - half, y_m + half)
            x1, y1 = self._world_to_canvas(x_m + half, y_m - half)
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill=OBSTACLE_FILL, outline=OBSTACLE_OUTLINE, width=2,
            )

    def _draw_policy_arrows(self) -> None:
        if self.iteration == 0:
            return
        arrow_len = self.world.spacing_m * 0.28
        arrow_w = max(2.5, self._meters_to_pixels(0.010))
        head = (max(8.0, self._meters_to_pixels(0.020)), max(10.0, self._meters_to_pixels(0.024)), max(5.0, self._meters_to_pixels(0.014)))
        y_off = self.world.spacing_m * 0.12

        pol, head_map = self._cell_policy_and_heading()
        for cell, action in pol.items():
            if action is None or cell == self.world.goal:
                continue
            x_m, y_m = self.world.world_xy(cell)
            draw_h = head_map.get(cell, Heading.N)

            if action == OrientedAction.FORWARD:
                ux, uy = _HEADING_UNIT_VEC[draw_h]
                s = self._world_to_canvas(x_m - ux * arrow_len / 2, y_m - uy * arrow_len / 2 - y_off)
                e = self._world_to_canvas(x_m + ux * arrow_len / 2, y_m + uy * arrow_len / 2 - y_off)
                self.canvas.create_line(
                    *s, *e, fill=ARROW_COLOR, width=arrow_w, arrow=tk.LAST,
                    arrowshape=head, capstyle=tk.ROUND,
                )
            else:
                arc = _oriented_turn_arc_world_xy(
                    x_m, y_m, draw_h, action == OrientedAction.TURN_LEFT, self.world.spacing_m * 0.13,
                )
                flat: list[float] = []
                for wx, wy in arc:
                    px, py = self._world_to_canvas(wx, wy)
                    flat.extend([px, py])
                self.canvas.create_line(
                    *flat, fill=ARROW_COLOR, width=arrow_w, smooth=True, arrow=tk.LAST, arrowshape=head,
                )

    def _draw_start_and_goal(self) -> None:
        r_px = max(10.0, self._meters_to_pixels(0.055))
        sx, sy = self._world_to_canvas(*self.world.world_xy(self.world.start))
        self.canvas.create_oval(sx - r_px, sy - r_px, sx + r_px, sy + r_px, outline=START_OUTLINE, width=4)
        self.canvas.create_text(sx, sy + r_px + 12, text="S", fill=START_OUTLINE, font=("Arial", 11, "bold"))
        ux, uy = _HEADING_UNIT_VEC[self.world.start_heading]
        tick = self._meters_to_pixels(self.world.spacing_m * 0.22)
        self.canvas.create_line(sx, sy, sx + ux * tick, sy - uy * tick, fill=START_OUTLINE, width=4, capstyle=tk.ROUND)

        gx, gy = self._world_to_canvas(*self.world.world_xy(self.world.goal))
        self.canvas.create_oval(gx - r_px, gy - r_px, gx + r_px, gy + r_px, fill=GOAL_FILL, outline=GOAL_OUTLINE, width=3)
        self.canvas.create_text(gx, gy, text="G", fill="white", font=("Arial", 13, "bold"))
