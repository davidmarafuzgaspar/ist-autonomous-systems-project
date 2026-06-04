from __future__ import annotations

import time
import tkinter as tk

from . import ui_theme as ui
from .value_iteration import ValueIteration, rollout_greedy_policy
from .world import (
    GAMMA_DEFAULT,
    MAX_ITERATIONS_DEFAULT,
    THETA_DEFAULT,
    GridAction,
    GridCell,
    Heading,
    IntersectionWorld,
    PoseState,
)

LINE_WIDTH_M = 0.06
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.10

_GRID_DELTA_RC: dict[Heading, tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
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
        self.window.title("Value Iteration (slip)")
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
        self._update_policy_path_label()
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
        self.change_world_requested = False
        self.window.mainloop()
        return self.change_world_requested

    def _build_sidebar(self) -> None:
        title = ui.label(self.sidebar, "Value Iteration (slip)")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w", pady=(0, 4))
        ui.label(
            self.sidebar,
            "Each step updates expected V(s) under forward slip. Pick a heading: each cell "
            "shows V; after converge, blue arrows = move / facing. Green path = greedy rollout "
            "from start with that initial heading (intended branch).",
            muted=True,
        ).pack(anchor="w", pady=(0, 10))

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
        self._build_policy_heading_panel()
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

    def _build_policy_heading_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Heading (per-cell π + path from start)", muted=False).pack(anchor="w")
        self.policy_heading_var = tk.StringVar(
            master=self.window,
            value=self.world.start_heading.name,
        )
        row = tk.Frame(self.sidebar, bg=ui.BG)
        row.pack(anchor="w", pady=4)
        for name in ("N", "E", "S", "W"):
            ui.radio(
                row,
                name,
                self.policy_heading_var,
                name,
                command=self._on_policy_heading_changed,
            ).pack(side=tk.LEFT, padx=(0, 8))
        ui.label(
            self.sidebar,
            "Blue arrow = where to go (move dir.); turns show new facing. Change heading to compare.",
            muted=True,
        ).pack(anchor="w")
        self.policy_path_label = ui.label(self.sidebar, "", muted=True)
        self.policy_path_label.pack(anchor="w", pady=(0, 4))

    def _view_heading(self) -> Heading:
        return Heading.from_str(self.policy_heading_var.get())

    def _on_policy_heading_changed(self) -> None:
        self._update_policy_path_label()
        self._draw()

    def _show_final_policy(self) -> bool:
        return self.converged

    def _update_policy_path_label(self) -> None:
        if not self._show_final_policy():
            if self.iteration == 0:
                self.policy_path_label.config(text="V = 0 everywhere. Press Next step.")
            else:
                delta_str = (
                    "∞"
                    if self.last_delta == float("inf")
                    else f"{self.last_delta:.4f}"
                )
                self.policy_path_label.config(
                    text=f"V still changing (Δ={delta_str}). Policy/path when converged.",
                )
            return
        start = PoseState(self.world.start, self._view_heading())
        path = rollout_greedy_policy(self.world, self.policy, start)
        if len(path) <= 1:
            self.policy_path_label.config(text="Path: blocked under this policy")
            return
        if path[-1].cell == self.world.goal:
            self.policy_path_label.config(
                text=f"Path: goal in {len(path) - 1} step(s), heading {start.heading.name}",
            )
        else:
            self.policy_path_label.config(
                text=f"Path: stops after {len(path) - 1} step(s) (loop / block)",
            )

    def _build_algorithm_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Update rule", muted=True).pack(anchor="w")
        self.algo_label = ui.label(self.sidebar, f"Active: {self._algorithm_name()}")
        self.algo_label.pack(anchor="w", pady=(0, 4))
        algo = tk.Frame(self.sidebar, bg=ui.BG)
        algo.pack(anchor="w", pady=(0, 4))
        for text, val in (("Gauss-Seidel", "gauss"), ("Jacobi", "jacobi")):
            ui.radio(
                algo,
                text,
                self.algorithm_var,
                val,
                command=self._on_algorithm_change,
            ).pack(side=tk.LEFT, padx=(0, 10))

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
        grid.pack(fill="x", pady=(0, 4))
        p_int, p_left, p_right = self.world.normalized_slip_probs()
        specs: list[tuple[str, str, float]] = [
            ("gamma", "γ", self.gamma),
            ("goal_reward", "goal", self.world.goal_reward),
            ("illegal_move_reward", "illegal", self.world.illegal_move_reward),
            ("reward_straight", "straight", self.world.reward_straight),
            ("reward_turn_right", "turn R", self.world.reward_turn_right),
            ("reward_turn_left", "turn L", self.world.reward_turn_left),
            ("reward_turn_around", "turn A", self.world.reward_turn_around),
            ("slip_prob_intended", "% intended", p_int * 100.0),
            ("slip_prob_left", "% slip left", p_left * 100.0),
            ("slip_prob_right", "% slip right", p_right * 100.0),
        ]
        for row, (key, lbl, value) in enumerate(specs):
            ui.label(grid, lbl, muted=True).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
            ent = ui.entry(grid, width=7)
            ent.insert(0, self._format_param(value))
            ent.grid(row=row, column=1, sticky="w", pady=2)
            self.param_entries[key] = ent

        ui.button(self.sidebar, "Apply parameters", self._on_apply_parameters).pack(fill="x", pady=2)
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

    def _parse_slip_percent(self, key: str) -> float:
        raw = float(self.param_entries[key].get())
        if raw < 0.0:
            raise ValueError(f"{key}: must be ≥ 0")
        return raw / 100.0 if raw > 1.0 else raw

    def _on_apply_parameters(self) -> None:
        try:
            new_gamma = float(self.param_entries["gamma"].get())
            new_goal = float(self.param_entries["goal_reward"].get())
            new_illegal = float(self.param_entries["illegal_move_reward"].get())
            new_straight = float(self.param_entries["reward_straight"].get())
            new_tr = float(self.param_entries["reward_turn_right"].get())
            new_tl = float(self.param_entries["reward_turn_left"].get())
            new_ta = float(self.param_entries["reward_turn_around"].get())
            slip_int = self._parse_slip_percent("slip_prob_intended")
            slip_left = self._parse_slip_percent("slip_prob_left")
            slip_right = self._parse_slip_percent("slip_prob_right")
        except ValueError as exc:
            self._set_param_message(f"Invalid number: {exc}", error=True)
            return

        if not 0.0 < new_gamma <= 1.0:
            self._set_param_message("γ must be in (0, 1]", error=True)
            return
        if slip_int + slip_left + slip_right <= 0.0:
            self._set_param_message("Slip weights must sum to > 0", error=True)
            return

        self.gamma = new_gamma
        self.world.goal_reward = new_goal
        self.world.illegal_move_reward = new_illegal
        self.world.reward_straight = new_straight
        self.world.reward_turn_right = new_tr
        self.world.reward_turn_left = new_tl
        self.world.reward_turn_around = new_ta
        self.world.slip_prob_intended = slip_int
        self.world.slip_prob_left = slip_left
        self.world.slip_prob_right = slip_right
        self.vi = self._make_vi()
        self._on_reset()
        self._set_param_message("Parameters updated")

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

    def _on_step(self) -> None:
        if self.converged or self.iteration >= self.max_iterations:
            return
        self.values, self.last_delta = self.vi.step(self.values)
        self.iteration += 1
        if self.last_delta < self.theta:
            self.converged = True
        self.policy = self.vi.greedy_policy(self.values)
        self._update_policy_path_label()
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
        while not self.converged and self.iteration < self.max_iterations:
            self._on_step()
            self.window.update_idletasks()
        self.time_label.config(text=f"Time: {elapsed_ms:.0f} ms")
        self._update_policy_path_label()

    def _on_reset(self) -> None:
        self.values = self.vi.initial_values()
        self.policy = self.vi.greedy_policy(self.values)
        self.iteration = 0
        self.last_delta = float("inf")
        self.converged = False
        self.time_label.config(text="Time: —")
        self._update_policy_path_label()
        self._draw()

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_value_overlays()
        self._draw_policy_path()
        self._draw_obstacles()
        self._draw_start_and_goal()
        self._update_sidebar_labels()

    def _update_sidebar_labels(self) -> None:
        self.iter_label.config(
            text=f"Iteration: {self.iteration}  ({self._algorithm_name()})",
        )
        delta_str = "∞" if self.last_delta == float("inf") else f"{self.last_delta:.4f}"
        self.delta_label.config(text=f"Delta (max |ΔV|): {delta_str}")
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
        if self.iteration == 0:
            return
        view_h = self._view_heading()
        box_half_m = self.world.spacing_m * 0.42
        for row in range(self.world.rows):
            for col in range(self.world.cols):
                cell = GridCell(row, col)
                if self.world.is_obstacle(cell):
                    continue
                value = self.values.get(PoseState(cell, view_h), 0.0)
                x_m, y_m = self.world.world_xy(cell)
                x0, y0 = self._world_to_canvas(x_m - box_half_m, y_m + box_half_m)
                x1, y1 = self._world_to_canvas(x_m + box_half_m, y_m - box_half_m)
                fill = self._value_to_color(value)
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="", width=0)
                cx, cy = self._world_to_canvas(x_m, y_m)
                self.canvas.create_text(
                    cx,
                    cy,
                    text=self._format_value(value),
                    fill="#333" if self._is_light(fill) else "#fff",
                    font=(ui.FONT[0], 8, "bold"),
                )
                if self._show_final_policy() and cell != self.world.goal:
                    action = self.policy.get(PoseState(cell, view_h))
                    if action is not None:
                        self._draw_policy_arrow(cx, cy, view_h, action)

    @staticmethod
    def _heading_after_action(heading: Heading, action: GridAction) -> Heading:
        if action == GridAction.TURN_RIGHT:
            return heading.turn_right()
        if action == GridAction.TURN_LEFT:
            return heading.turn_left()
        if action == GridAction.TURN_AROUND:
            return heading.turn_right().turn_right()
        return heading

    def _draw_policy_arrow(
        self,
        cx: float,
        cy: float,
        view_h: Heading,
        action: GridAction,
    ) -> None:
        if action == GridAction.STRAIGHT:
            move_h = view_h
            width = 2.5
        else:
            move_h = self._heading_after_action(view_h, action)
            width = 2.0
        dr, dc = _GRID_DELTA_RC[move_h]
        tick = max(12.0, self._meters_to_pixels(self.world.spacing_m * 0.22))
        y_off = 11.0
        self.canvas.create_line(
            cx,
            cy + y_off,
            cx + dc * tick,
            cy + y_off + dr * tick,
            fill=ui.POLICY,
            width=width,
            arrow=tk.LAST,
            arrowshape=(7, 9, 4),
            capstyle=tk.ROUND,
        )

    def _draw_policy_path(self) -> None:
        if not self._show_final_policy():
            return
        start = PoseState(self.world.start, self._view_heading())
        path = rollout_greedy_policy(self.world, self.policy, start)
        if len(path) < 2:
            return
        pts = [self._world_to_canvas(*self.world.world_xy(s.cell)) for s in path]
        flat = [c for p in pts for c in p]
        self.canvas.create_line(*flat, fill=ui.CELL_START, width=3, smooth=False)

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
