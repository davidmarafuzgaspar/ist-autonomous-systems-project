"""Interactive VI viewer."""

from __future__ import annotations

import math
import time
import tkinter as tk

from . import ui_theme as ui
from .value_iteration import ValueIteration
from .world import (
    GAMMA_DEFAULT,
    MAX_ITERATIONS_DEFAULT,
    THETA_DEFAULT,
    GridAction,
    Heading,
    IntersectionWorld,
    PoseState,
)

LINE_WIDTH_M = 0.06
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.10

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
        gamma: float = GAMMA_DEFAULT,
        theta: float = THETA_DEFAULT,
        max_iterations: int = MAX_ITERATIONS_DEFAULT,
        synchronous: bool = False,
        canvas_size_px: int = 720,
        sidebar_width_px: int = 260,
    ) -> None:
        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.synchronous = synchronous
        self.canvas_size_px = canvas_size_px
        self.sidebar_width_px = sidebar_width_px
        self.change_world_requested = False
        self.param_entries: dict[str, tk.Entry] = {}
        self.param_message: tk.Label | None = None

        self.vi = self._make_vi()
        self.values: dict[PoseState, float] = self.vi.initial_values()
        self.policy: dict[PoseState, GridAction | None] = self.vi.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta = float("inf")
        self.converged = False

        self.window = tk.Tk()
        self.window.title("Value Iteration")
        ui.setup(self.window)

        self.algorithm_var = tk.StringVar(
            master=self.window,
            value="jacobi" if synchronous else "gauss",
        )

        body = tk.Frame(self.window, bg=ui.BG)
        body.pack(fill="both", expand=True, padx=12, pady=12)

        self.canvas = tk.Canvas(
            body,
            width=canvas_size_px,
            height=canvas_size_px,
            bg=ui.CANVAS_BG,
            highlightthickness=0,
        )
        self.canvas.pack(side=tk.LEFT, padx=(0, 12))

        self.sidebar = tk.Frame(body, bg=ui.BG, width=sidebar_width_px)
        self.sidebar.pack(side=tk.RIGHT, fill="y")

        self._build_sidebar()
        self._draw()

    def _make_vi(self) -> ValueIteration:
        return ValueIteration(
            self.world,
            gamma=self.gamma,
            theta=self.theta,
            max_iterations=self.max_iterations,
            synchronous=self.synchronous,
        )

    def run(self) -> bool:
        """Block until close. True if Change world was pressed."""
        self.change_world_requested = False
        self.window.mainloop()
        return self.change_world_requested

    def _build_sidebar(self) -> None:
        title = ui.label(self.sidebar, "Value Iteration")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w", pady=(0, 10))

        self.iter_label = ui.label(self.sidebar, "Iteration: 0")
        self.iter_label.pack(anchor="w")
        self.delta_label = ui.label(self.sidebar, "Delta: —")
        self.delta_label.pack(anchor="w")
        self.time_label = ui.label(self.sidebar, "Time: —")
        self.time_label.pack(anchor="w")
        self.status_label = ui.label(self.sidebar, "Status: ready")
        self.status_label.pack(anchor="w", pady=(0, 10))

        ui.button(self.sidebar, "Next step", self._on_step).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Run to convergence", self._on_run_to_convergence, primary=True).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Reset", self._on_reset).pack(fill="x", pady=2)

        self._build_algorithm_panel()
        self._build_parameter_panel()

        ui.line(self.sidebar)
        ui.label(
            self.sidebar,
            f"{self.world.rows}×{self.world.cols} · "
            f"S({self.world.start.row},{self.world.start.col}) · "
            f"G({self.world.goal.row},{self.world.goal.col})",
            muted=True,
        ).pack(anchor="w", pady=(0, 8))
        ui.button(self.sidebar, "Change world", self._on_change_world).pack(fill="x")

    def _on_change_world(self) -> None:
        self.change_world_requested = True
        self.window.destroy()

    def _algorithm_name(self) -> str:
        return "Jacobi" if self.synchronous else "Gauss-Seidel"

    def _build_algorithm_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Update rule", muted=True).pack(anchor="w")
        self.algo_label = ui.label(self.sidebar, f"Active: {self._algorithm_name()}")
        self.algo_label.pack(anchor="w", pady=(0, 4))
        algo = tk.Frame(self.sidebar, bg=ui.BG)
        algo.pack(anchor="w", pady=(0, 4))
        for text, val in (("Gauss-Seidel", "gauss"), ("Jacobi", "jacobi")):
            rb = ui.radio(algo, text, self.algorithm_var, val)
            rb.configure(command=self._on_algorithm_change)
            rb.pack(side=tk.LEFT, padx=(0, 10))

    def _on_algorithm_change(self) -> None:
        self.synchronous = self.algorithm_var.get() == "jacobi"
        self.vi = self._make_vi()
        self.algo_label.config(text=f"Active: {self._algorithm_name()}")
        self._on_reset()
        self._set_param_message("")

    def _build_parameter_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Rewards", muted=True).pack(anchor="w")

        grid = tk.Frame(self.sidebar, bg=ui.BG)
        grid.pack(fill="x", pady=(0, 6))
        specs: list[tuple[str, str, float]] = [
            ("gamma", "γ", self.gamma),
            ("goal_reward", "goal", self.world.goal_reward),
            ("illegal_move_reward", "illegal", self.world.illegal_move_reward),
            ("reward_straight", "straight", self.world.reward_straight),
            ("reward_turn_right", "turn R", self.world.reward_turn_right),
            ("reward_turn_left", "turn L", self.world.reward_turn_left),
            ("reward_turn_around", "turn A", self.world.reward_turn_around),
        ]
        for row, (key, lbl, value) in enumerate(specs):
            ui.label(grid, lbl, muted=True).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
            ent = ui.entry(grid, width=7)
            ent.insert(0, self._format_param(value))
            ent.grid(row=row, column=1, sticky="w", pady=2)
            self.param_entries[key] = ent

        ui.button(self.sidebar, "Apply rewards", self._on_apply_parameters).pack(fill="x", pady=2)
        self.param_message = ui.label(self.sidebar, "", muted=True)
        self.param_message.pack(anchor="w", pady=(4, 0))

    @staticmethod
    def _format_param(value: float) -> str:
        if float(value).is_integer():
            return str(int(value))
        return f"{value:g}"

    def _set_param_message(self, text: str, *, error: bool = False) -> None:
        if self.param_message is not None:
            self.param_message.config(text=text, fg="#f87171" if error else ui.MUTED)

    def _on_apply_parameters(self) -> None:
        try:
            new_gamma = float(self.param_entries["gamma"].get())
            new_goal = float(self.param_entries["goal_reward"].get())
            new_illegal = float(self.param_entries["illegal_move_reward"].get())
            new_straight = float(self.param_entries["reward_straight"].get())
            new_tr = float(self.param_entries["reward_turn_right"].get())
            new_tl = float(self.param_entries["reward_turn_left"].get())
            new_ta = float(self.param_entries["reward_turn_around"].get())
        except ValueError as exc:
            self._set_param_message(f"Invalid number: {exc}", error=True)
            return

        if not 0.0 < new_gamma <= 1.0:
            self._set_param_message("γ must be in (0, 1]", error=True)
            return

        self.gamma = new_gamma
        self.world.goal_reward = new_goal
        self.world.illegal_move_reward = new_illegal
        self.world.reward_straight = new_straight
        self.world.reward_turn_right = new_tr
        self.world.reward_turn_left = new_tl
        self.world.reward_turn_around = new_ta
        self.vi = self._make_vi()
        self._on_reset()
        self._set_param_message("Rewards updated")

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

    def _row_line_centers(self) -> list[float]:
        half_span = (self.world.rows - 1) / 2.0
        return [(index - half_span) * self.world.spacing_m for index in range(self.world.rows)]

    def _col_line_centers(self) -> list[float]:
        half_span = (self.world.cols - 1) / 2.0
        return [(index - half_span) * self.world.spacing_m for index in range(self.world.cols)]

    def _cell_policy_and_heading(
        self,
    ) -> tuple[dict[GridCell, GridAction | None], dict[GridCell, Heading]]:
        pol = self.world.aggregated_policy_per_cell(self.values, self.gamma)
        head = self.world.display_heading_map_for_cell_policy(pol, self.values, self.gamma)
        return pol, head

    def _on_step(self) -> None:
        if self.converged or self.iteration >= self.max_iterations:
            return
        self.values, self.last_delta = self.vi.step(self.values)
        self.iteration += 1
        if self.last_delta < self.theta:
            self.converged = True
        self.policy = self.vi.greedy_policy(self.values)
        self._draw()

    def _on_run_to_convergence(self) -> None:
        if self.converged:
            return
        algo_values = dict(self.values)
        algo_iter = self.iteration
        start = time.perf_counter()
        while algo_iter < self.max_iterations:
            algo_values, delta = self.vi.step(algo_values)
            algo_iter += 1
            if delta < self.theta:
                break
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        iters_done = algo_iter - self.iteration
        while not self.converged and self.iteration < self.max_iterations:
            self._on_step()
            self.window.update_idletasks()
        self.time_label.config(text=f"Time: {elapsed_ms:.0f} ms ({iters_done} it)")

    def _on_reset(self) -> None:
        self.values = self.vi.initial_values()
        self.policy = self.vi.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta = float("inf")
        self.converged = False
        self.time_label.config(text="Time: —")
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
        self.iter_label.config(
            text=f"Iteration: {self.iteration}  ({self._algorithm_name()})",
        )
        delta_str = "∞" if self.last_delta == float("inf") else f"{self.last_delta:.4f}"
        self.delta_label.config(text=f"Delta: {delta_str}")
        if self.iteration == 0:
            self.status_label.config(text="Status: ready", fg=ui.MUTED)
        elif self.converged:
            self.status_label.config(text="Status: converged", fg=ui.ACCENT)
        else:
            self.status_label.config(text="Status: running", fg=ui.FG)

    def _draw_board(self) -> None:
        half = self._half_extent_m
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=ui.BOARD, outline=ui.BORDER, width=1)
        line_width_px = max(1.0, self._meters_to_pixels(LINE_WIDTH_M))
        for center in self._col_line_centers():
            x0, y0 = self._world_to_canvas(center, half)
            x1, y1 = self._world_to_canvas(center, -half)
            self.canvas.create_line(x0, y0, x1, y1, fill=ui.LINE, width=line_width_px)
        for center in self._row_line_centers():
            x0, y0 = self._world_to_canvas(-half, center)
            x1, y1 = self._world_to_canvas(half, center)
            self.canvas.create_line(x0, y0, x1, y1, fill=ui.LINE, width=line_width_px)

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
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="", width=0)
            cx, cy = self._world_to_canvas(x_m, y_m)
            self.canvas.create_text(
                cx, cy,
                text=self._format_value(value),
                fill="#333" if self._is_light(fill) else "#fff",
                font=(ui.FONT[0], 8, "bold"),
            )

    def _value_to_color(self, value: float) -> str:
        if value > 0:
            t = min(1.0, value / ui.POS_SCALE_MAX)
            return "#dbeafe" if t < 0.5 else "#93c5fd"
        if value < 0:
            return "#fee2e2"
        return ui.CELL_FREE

    @staticmethod
    def _is_light(hex_color: str) -> bool:
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return 0.299 * r + 0.587 * g + 0.114 * b > 140.0

    @staticmethod
    def _format_value(value: float) -> str:
        return f"{value:.0f}" if abs(value) >= 100.0 else f"{value:.1f}"

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self.world.obstacles:
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - half, y_m + half)
            x1, y1 = self._world_to_canvas(x_m + half, y_m - half)
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill=ui.OBSTACLE, outline=ui.BORDER, width=1,
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

            if action == GridAction.STRAIGHT:
                move_h = self.world.movement_heading_for_action(draw_h, action)
                ux, uy = _HEADING_UNIT_VEC[move_h]
                s = self._world_to_canvas(x_m - ux * arrow_len / 2, y_m - uy * arrow_len / 2 - y_off)
                e = self._world_to_canvas(x_m + ux * arrow_len / 2, y_m + uy * arrow_len / 2 - y_off)
                self.canvas.create_line(
                    *s, *e, fill=ui.POLICY, width=arrow_w, arrow=tk.LAST,
                    arrowshape=head, capstyle=tk.ROUND,
                )
            elif action == GridAction.TURN_AROUND:
                cx, cy = self._world_to_canvas(x_m, y_m - y_off)
                self.canvas.create_text(
                    cx, cy, text="A", fill=ui.POLICY, font=ui.FONT_BOLD,
                )
            else:
                turn_left = action == GridAction.TURN_LEFT
                arc = _oriented_turn_arc_world_xy(
                    x_m, y_m, draw_h, turn_left, self.world.spacing_m * 0.13,
                )
                flat: list[float] = []
                for wx, wy in arc:
                    px, py = self._world_to_canvas(wx, wy)
                    flat.extend([px, py])
                self.canvas.create_line(
                    *flat, fill=ui.POLICY, width=arrow_w, smooth=True, arrow=tk.LAST, arrowshape=head,
                )

    def _draw_start_and_goal(self) -> None:
        r_px = max(10.0, self._meters_to_pixels(0.055))
        sx, sy = self._world_to_canvas(*self.world.world_xy(self.world.start))
        self.canvas.create_oval(
            sx - r_px, sy - r_px, sx + r_px, sy + r_px,
            outline=ui.CELL_START, width=2, fill="",
        )
        ux, uy = _HEADING_UNIT_VEC[self.world.start_heading]
        tick = self._meters_to_pixels(self.world.spacing_m * 0.22)
        self.canvas.create_line(
            sx, sy, sx + ux * tick, sy - uy * tick,
            fill=ui.CELL_START, width=2, capstyle=tk.ROUND,
        )

        gx, gy = self._world_to_canvas(*self.world.world_xy(self.world.goal))
        self.canvas.create_oval(
            gx - r_px, gy - r_px, gx + r_px, gy + r_px,
            fill=ui.CELL_GOAL, outline=ui.BORDER, width=1,
        )
        self.canvas.create_text(gx, gy, text="G", fill="#fff", font=ui.FONT_BOLD)
