"""Map setup + training/iteration viewer (Tk)."""

from __future__ import annotations

from __future__ import annotations

import sys
import tkinter as tk
import tkinter.font as tkfont
import types
from typing import Callable

BG = "#1e1e1e"
BG2 = "#2a2a2a"
FG = "#f0f0f0"
MUTED = "#9ca3af"
ACCENT = "#4a9eff"
BORDER = "#404040"

CANVAS_BG = "#252525"
BOARD = "#fafafa"
LINE = "#d4d4d4"
CELL_FREE = "#ffffff"
CELL_BLOCK = "#6b7280"
CELL_START = "#22c55e"
CELL_GOAL = "#4a9eff"
POLICY = "#4a9eff"
OBSTACLE = "#6b7280"

POS_SCALE_MAX = 150.0
NEG_SCALE_MAX = 60.0

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 11, "bold")
FONT_TITLE = ("Segoe UI", 14, "bold")


def setup(root: tk.Misc) -> None:
    global FONT, FONT_BOLD, FONT_TITLE
    root.configure(bg=BG)
    try:
        names = set(tkfont.families())
        for family in ("Segoe UI", "Ubuntu", "Helvetica Neue", "Arial"):
            if family in names:
                FONT = (family, 10)
                FONT_BOLD = (family, 11, "bold")
                FONT_TITLE = (family, 14, "bold")
                break
    except (tk.TclError, RuntimeError):
        if sys.platform.startswith("linux"):
            FONT = ("Ubuntu", 10)
            FONT_BOLD = ("Ubuntu", 11, "bold")
            FONT_TITLE = ("Ubuntu", 14, "bold")


def label(parent: tk.Misc, text: str, *, muted: bool = False) -> tk.Label:
    return tk.Label(
        parent,
        text=text,
        font=FONT,
        fg=MUTED if muted else FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        anchor="w",
    )


def button(parent: tk.Misc, text: str, command: Callable[[], None], *, primary: bool = False) -> tk.Button:
    return tk.Button(
        parent,
        text=text,
        command=command,
        font=FONT_BOLD if primary else FONT,
        bg=ACCENT if primary else BG2,
        fg="#ffffff" if primary else FG,
        activebackground="#3d8bfd" if primary else "#353535",
        activeforeground="#ffffff",
        relief="flat",
        padx=12,
        pady=6,
        cursor="hand2",
    )


def entry(parent: tk.Misc, *, width: int = 6) -> tk.Entry:
    return tk.Entry(
        parent,
        width=width,
        font=FONT,
        bg=BG2,
        fg=FG,
        insertbackground=FG,
        relief="flat",
        highlightthickness=1,
        highlightbackground=BORDER,
        highlightcolor=ACCENT,
    )


def radio(
    parent: tk.Misc,
    text: str,
    variable: tk.StringVar,
    value: str,
    *,
    command: Callable[[], None] | None = None,
) -> tk.Radiobutton:
    return tk.Radiobutton(
        parent,
        text=text,
        variable=variable,
        value=value,
        command=command,
        font=FONT,
        fg=FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        selectcolor=BG2,
        activebackground=BG,
        activeforeground=FG,
        anchor="w",
    )


def line(parent: tk.Misc) -> None:
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", pady=10)


ui = types.SimpleNamespace(
    ACCENT=ACCENT,
    BG=BG,
    BG2=BG2,
    BOARD=BOARD,
    BORDER=BORDER,
    CANVAS_BG=CANVAS_BG,
    CELL_BLOCK=CELL_BLOCK,
    CELL_FREE=CELL_FREE,
    CELL_GOAL=CELL_GOAL,
    CELL_START=CELL_START,
    FG=FG,
    FONT=FONT,
    FONT_BOLD=FONT_BOLD,
    FONT_TITLE=FONT_TITLE,
    LINE=LINE,
    MUTED=MUTED,
    NEG_SCALE_MAX=NEG_SCALE_MAX,
    OBSTACLE=OBSTACLE,
    POLICY=POLICY,
    POS_SCALE_MAX=POS_SCALE_MAX,
    button=button,
    entry=entry,
    label=label,
    line=line,
    radio=radio,
    setup=setup,
)


import tkinter as tk
from tkinter import messagebox

from .model import (
    MAX_GRID_SIZE,
    MIN_GRID_SIZE,
    OBSTACLE_CELL,
    FREE_CELL,
    GridCell,
    Heading,
    IntersectionWorld,
    empty_grid,
)

MODE_OBSTACLE = "obstacle"
MODE_START = "start"
MODE_GOAL = "goal"

# Canvas axes: row 0 at top; matches grid row/col deltas for N,E,S,W.
_HEADING_CANVAS_VEC: dict[str, tuple[float, float]] = {
    "N": (0.0, -1.0),
    "E": (1.0, 0.0),
    "S": (0.0, 1.0),
    "W": (-1.0, 0.0),
}


class MapSetupDialog:
    def __init__(
        self,
        *,
        initial_rows: int = 5,
        initial_cols: int = 5,
        initial_world: IntersectionWorld | None = None,
    ) -> None:
        self._result: IntersectionWorld | None = None
        if initial_world is not None:
            self._rows = initial_world.rows
            self._cols = initial_world.cols
            self._grid = [list(row) for row in initial_world.grid]
            self._start = initial_world.start
            self._goal = initial_world.goal
            self._initial_heading = initial_world.start_heading.name
            self._reward_defaults = initial_world
        else:
            self._reward_defaults = None
            self._rows = initial_rows
            self._cols = initial_cols
            self._grid = empty_grid(self._rows, self._cols)
            self._start = GridCell(0, 0)
            self._goal = GridCell(self._rows - 1, self._cols - 1)
            self._initial_heading = "S"
        self._canvas_px = 480

        self.root = tk.Tk()
        self.root.title("Map setup")
        ui.setup(self.root)

        root = tk.Frame(self.root, bg=ui.BG, padx=16, pady=16)
        root.pack()

        title = ui.label(root, "Map setup")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w")
        ui.label(root, "Click cells to edit. Then continue.", muted=True).pack(anchor="w", pady=(2, 12))

        top = tk.Frame(root, bg=ui.BG)
        top.pack(fill="x", pady=(0, 10))
        ui.label(top, "Rows").pack(side=tk.LEFT)
        self.rows_entry = ui.entry(top, width=4)
        self.rows_entry.insert(0, str(self._rows))
        self.rows_entry.pack(side=tk.LEFT, padx=(4, 12))
        ui.label(top, "Cols").pack(side=tk.LEFT)
        self.cols_entry = ui.entry(top, width=4)
        self.cols_entry.insert(0, str(self._cols))
        self.cols_entry.pack(side=tk.LEFT, padx=(4, 12))
        ui.button(top, "Apply", self._apply_size).pack(side=tk.LEFT)

        self.mode_var = tk.StringVar(master=self.root, value=MODE_OBSTACLE)
        modes = tk.Frame(root, bg=ui.BG)
        modes.pack(anchor="w", pady=(0, 8))
        for text, val in (("Obstacle", MODE_OBSTACLE), ("Start", MODE_START), ("Goal", MODE_GOAL)):
            ui.radio(modes, text, self.mode_var, val).pack(side=tk.LEFT, padx=(0, 12))

        head = tk.Frame(root, bg=ui.BG)
        head.pack(anchor="w", pady=(0, 10))
        ui.label(head, "Heading:").pack(side=tk.LEFT, padx=(0, 8))
        self.heading_var = tk.StringVar(master=self.root, value=self._initial_heading)
        for h in ("N", "E", "S", "W"):
            rb = ui.radio(head, h, self.heading_var, h)
            rb.configure(command=self._redraw)
            rb.pack(side=tk.LEFT, padx=(0, 6))

        self.canvas = tk.Canvas(root, width=self._canvas_px, height=self._canvas_px, bg=ui.CANVAS_BG, highlightthickness=0)
        self.canvas.pack(pady=(0, 12))
        self.canvas.bind("<Button-1>", self._on_click)

        btns = tk.Frame(root, bg=ui.BG)
        btns.pack(fill="x")
        ui.button(btns, "Clear obstacles", self._clear_obstacles).pack(side=tk.LEFT)
        ui.button(btns, "Continue", self._on_confirm, primary=True).pack(side=tk.RIGHT)
        ui.button(btns, "Cancel", self.root.destroy).pack(side=tk.RIGHT, padx=(0, 8))

        self._redraw()

    def run(self) -> IntersectionWorld | None:
        self.root.mainloop()
        return self._result

    def _parse_size(self) -> tuple[int, int] | None:
        try:
            rows, cols = int(self.rows_entry.get()), int(self.cols_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Rows and cols must be integers.")
            return None
        if not (MIN_GRID_SIZE <= rows <= MAX_GRID_SIZE and MIN_GRID_SIZE <= cols <= MAX_GRID_SIZE):
            messagebox.showerror("Error", f"Size must be {MIN_GRID_SIZE}–{MAX_GRID_SIZE}.")
            return None
        return rows, cols

    def _apply_size(self) -> None:
        p = self._parse_size()
        if p is None:
            return
        self._rows, self._cols = p
        self._grid = empty_grid(self._rows, self._cols)
        self._start, self._goal = GridCell(0, 0), GridCell(self._rows - 1, self._cols - 1)
        self._redraw()

    def _clear_obstacles(self) -> None:
        for r in range(self._rows):
            for c in range(self._cols):
                if GridCell(r, c) not in (self._start, self._goal):
                    self._grid[r][c] = FREE_CELL
        self._redraw()

    def _cell_at(self, event: tk.Event) -> GridCell | None:
        pad = 16
        cell_px = (self._canvas_px - 2 * pad) / max(self._rows, self._cols)
        x0 = (self._canvas_px - cell_px * self._cols) / 2
        y0 = (self._canvas_px - cell_px * self._rows) / 2
        col = int((event.x - x0) / cell_px)
        row = int((event.y - y0) / cell_px)
        if 0 <= row < self._rows and 0 <= col < self._cols:
            return GridCell(row, col)
        return None

    def _on_click(self, event: tk.Event) -> None:
        cell = self._cell_at(event)
        if cell is None:
            return
        r, c = cell.row, cell.col
        mode = self.mode_var.get()
        if mode == MODE_OBSTACLE and cell not in (self._start, self._goal):
            self._grid[r][c] = FREE_CELL if self._grid[r][c] == OBSTACLE_CELL else OBSTACLE_CELL
        elif mode == MODE_START and self._grid[r][c] != OBSTACLE_CELL:
            self._start = cell
        elif mode == MODE_GOAL and self._grid[r][c] != OBSTACLE_CELL and cell != self._start:
            self._goal = cell
        self._redraw()

    def _on_confirm(self) -> None:
        if self._start == self._goal:
            messagebox.showerror("Error", "Start and goal must differ.")
            return
        kwargs: dict = {}
        if self._reward_defaults is not None:
            ref = self._reward_defaults
            kwargs = {
                "goal_reward": ref.goal_reward,
                "illegal_move_reward": ref.illegal_move_reward,
                "reward_straight": ref.reward_straight,
                "reward_turn_right": ref.reward_turn_right,
                "reward_turn_left": ref.reward_turn_left,
                "reward_turn_around": ref.reward_turn_around,
            }
        self._result = IntersectionWorld(
            grid=[list(row) for row in self._grid],
            start=self._start,
            goal=self._goal,
            start_heading=Heading.from_str(self.heading_var.get()),
            **kwargs,
        )
        self.root.destroy()

    def _redraw(self) -> None:
        self.canvas.delete("all")
        pad = 16
        cell_px = (self._canvas_px - 2 * pad) / max(self._rows, self._cols)
        x0 = (self._canvas_px - cell_px * self._cols) / 2
        y0 = (self._canvas_px - cell_px * self._rows) / 2
        for row in range(self._rows):
            for col in range(self._cols):
                cell = GridCell(row, col)
                x1, y1 = x0 + col * cell_px, y0 + row * cell_px
                x2, y2 = x1 + cell_px, y1 + cell_px
                if cell == self._start:
                    fill, mark = ui.CELL_START, "S"
                elif cell == self._goal:
                    fill, mark = ui.CELL_GOAL, "G"
                elif self._grid[row][col] == OBSTACLE_CELL:
                    fill, mark = ui.CELL_BLOCK, ""
                else:
                    fill, mark = ui.CELL_FREE, ""
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline=ui.BORDER)
                if cell == self._start:
                    self._draw_start_heading(x1, y1, x2, y2, cell_px)
                elif mark:
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=mark, fill="#fff", font=ui.FONT_BOLD)

    def _draw_start_heading(self, x1: float, y1: float, x2: float, y2: float, cell_px: float) -> None:
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        heading = self.heading_var.get()
        ux, uy = _HEADING_CANVAS_VEC.get(heading, (0.0, 1.0))
        self.canvas.create_text(cx, cy, text="S", fill="#fff", font=ui.FONT_BOLD)
        tick = cell_px * 0.32
        tip_x = cx + ux * tick
        tip_y = cy + uy * tick
        self.canvas.create_line(
            cx, cy, tip_x, tip_y,
            fill="#ffffff",
            width=2,
            arrow=tk.LAST,
            arrowshape=(5, 7, 3),
            capstyle=tk.ROUND,
        )
        label_off = cell_px * 0.22
        self.canvas.create_text(
            tip_x + ux * label_off,
            tip_y + uy * label_off,
            text=heading,
            fill="#ffffff",
            font=(ui.FONT[0], 9, "bold"),
        )


def run_map_setup(
    *,
    initial_rows: int = 5,
    initial_cols: int = 5,
    initial_world: IntersectionWorld | None = None,
) -> IntersectionWorld | None:
    return MapSetupDialog(
        initial_rows=initial_rows,
        initial_cols=initial_cols,
        initial_world=initial_world,
    ).run()


from .model import (
    GAMMA_DEFAULT,
    MAX_ITERATIONS_DEFAULT,
    THETA_DEFAULT,
    GridAction,
    GridCell,
    Heading,
    IntersectionWorld,
    PoseState,
    ValueIteration,
    rollout_greedy_policy,
)
import time
import tkinter as tk


LINE_WIDTH_M = 0.06
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.10

# Screen deltas (row 0 = top); matches map setup compass.
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
        title = ui.label(self.sidebar, "Value Iteration")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w", pady=(0, 4))
        ui.label(
            self.sidebar,
            "Each step updates V(s). Pick a heading: each cell shows V; after converge, "
            "blue arrows = move / facing. Green path = greedy rollout from start with "
            "that initial heading.",
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
        iters_done = algo_iter - self.iteration
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
        """Arrow on screen: straight = move that way; turn = face that way after turn."""
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
