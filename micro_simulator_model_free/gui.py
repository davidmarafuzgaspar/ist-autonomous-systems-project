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
ROBOT = "#e91e63"
TRAIL = "#ff80ab"

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
    ROBOT=ROBOT,
    TRAIL=TRAIL,
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
                    fill = ui.CELL_START
                elif cell == self._goal:
                    fill = ui.CELL_GOAL
                elif self._grid[row][col] == OBSTACLE_CELL:
                    fill = ui.CELL_BLOCK
                else:
                    fill = ui.CELL_FREE
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline=ui.BORDER)
                if cell == self._start:
                    self._draw_start_heading(x1, y1, x2, y2, cell_px)
                elif cell == self._goal:
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text="G", fill="#fff", font=ui.FONT_BOLD)

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
            fill="#ffffff", width=2, arrow=tk.LAST, arrowshape=(5, 7, 3), capstyle=tk.ROUND,
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
    ACTION_GLYPHS,
    ALPHA_DEFAULT,
    EPSILON_DECAY_DEFAULT,
    EPSILON_END_DEFAULT,
    EPSILON_START_DEFAULT,
    GAMMA_DEFAULT,
    MAX_STEPS_DEFAULT,
    NUM_EPISODES_DEFAULT,
    QLearningTrainer,
    GridAction,
    GridCell,
    Heading,
    IntersectionWorld,
    PoseState,
)
import tkinter as tk
from tkinter import messagebox, scrolledtext


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


class QLearningViewer:
    def __init__(
        self,
        world: IntersectionWorld,
        *,
        canvas_size_px: int = 720,
        sidebar_width_px: int = 280,
    ) -> None:
        self.world = world
        self.trainer = QLearningTrainer(world)
        self.trail: list[PoseState] = []
        self.policy_final_ready = False
        self.change_world_requested = False
        self.param_entries: dict[str, tk.Entry] = {}
        self.param_message: tk.Label | None = None
        self.canvas_size_px = canvas_size_px

        self.window = tk.Tk()
        self.window.title("Model-free Q-learning (solver visualizer)")
        ui.setup(self.window)

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
        self._refresh_policy_panel_state()
        self._draw()

    def run(self) -> bool:
        self.change_world_requested = False
        self.window.mainloop()
        return self.change_world_requested

    def _build_sidebar(self) -> None:
        title = ui.label(self.sidebar, "Model-free (solver)")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w", pady=(0, 4))

        ui.label(
            self.sidebar,
            "Train with Next step / Run episode. "
            "Pink dot = robot during training; pink trail = current episode.",
            muted=True,
        ).pack(anchor="w", pady=(0, 8))

        self.ep_label = ui.label(self.sidebar, "Episodes: 0 / not started")
        self.ep_label.pack(anchor="w")
        self.step_label = ui.label(self.sidebar, "Step: —")
        self.step_label.pack(anchor="w")
        self.eps_label = ui.label(self.sidebar, f"ε: {self.trainer.epsilon:.3f}")
        self.eps_label.pack(anchor="w")
        self.stats_label = ui.label(self.sidebar, "Goals reached: 0 / 0 episodes", muted=True)
        self.stats_label.pack(anchor="w")
        self.status_label = ui.label(self.sidebar, "Press Next step or Run episode")
        self.status_label.pack(anchor="w", pady=(0, 8))

        self.last_label = ui.label(self.sidebar, "", muted=True)
        self.last_label.pack(anchor="w", pady=(0, 8))

        ui.button(self.sidebar, "Next step", self._on_step).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Run episode", self._on_run_episode, primary=True).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Train all episodes", self._on_train_all).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Show policy (text)", self._on_show_policy).pack(fill="x", pady=2)
        ui.button(self.sidebar, "Reset Q-table", self._on_reset_q).pack(fill="x", pady=2)

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

    def _build_policy_heading_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Final policy (after all episodes)", muted=False).pack(anchor="w")
        self.policy_heading_var = tk.StringVar(value=self.world.start_heading.name)
        row = tk.Frame(self.sidebar, bg=ui.BG)
        row.pack(anchor="w", pady=4)
        for name in ("N", "E", "S", "W"):
            ui.radio(row, name, self.policy_heading_var, name, command=self._on_policy_heading_changed).pack(
                side=tk.LEFT,
                padx=(0, 8),
            )
        ui.label(
            self.sidebar,
            "After training: max Q per cell + blue arrow (where to go). "
            "Green path = rollout from start.",
            muted=True,
        ).pack(anchor="w")
        self.policy_path_label = ui.label(self.sidebar, "", muted=True)
        self.policy_path_label.pack(anchor="w", pady=(0, 4))

    def _selected_policy_heading(self) -> Heading:
        return Heading.from_str(self.policy_heading_var.get())

    def _read_num_episodes(self) -> int:
        try:
            value = int(self.param_entries["episodes"].get())
            return max(1, value)
        except (ValueError, KeyError):
            return self.trainer.num_episodes

    def _sync_training_params_from_ui(self) -> None:
        self.trainer.num_episodes = self._read_num_episodes()

    def _mark_policy_final_if_done(self) -> None:
        if self.trainer.training_finished():
            self.policy_final_ready = True
            self._refresh_policy_panel_state()

    def _refresh_policy_panel_state(self) -> None:
        if not self.policy_final_ready:
            self.policy_path_label.config(
                text="Policy on grid / text: available after all episodes finish.",
            )
            return
        self._update_policy_path_label()

    def _on_policy_heading_changed(self) -> None:
        if not self.policy_final_ready:
            return
        self._update_policy_path_label()
        self._draw()

    def _update_policy_path_label(self) -> None:
        if not self.policy_final_ready:
            return
        start = PoseState(self.world.start, self._selected_policy_heading())
        path = self.trainer.greedy_rollout(start)
        if len(path) <= 1:
            self.policy_path_label.config(text="Path: no steps (train first or blocked)")
            return
        reached = path[-1].cell == self.world.goal
        if reached:
            self.policy_path_label.config(
                text=f"Path: goal in {len(path) - 1} step(s) with heading {start.heading.name}",
            )
        else:
            self.policy_path_label.config(
                text=f"Path: stops after {len(path) - 1} step(s) (no goal / loop / block)",
            )

    def _build_parameter_panel(self) -> None:
        ui.line(self.sidebar)
        ui.label(self.sidebar, "Parameters (solver defaults)", muted=True).pack(anchor="w")

        fields = [
            ("alpha", str(ALPHA_DEFAULT)),
            ("gamma", str(GAMMA_DEFAULT)),
            ("epsilon_start", str(EPSILON_START_DEFAULT)),
            ("epsilon_end", str(EPSILON_END_DEFAULT)),
            ("epsilon_decay", str(EPSILON_DECAY_DEFAULT)),
            ("max_steps", str(MAX_STEPS_DEFAULT)),
            ("episodes", str(NUM_EPISODES_DEFAULT)),
            ("goal", str(self.world.goal_reward)),
            ("illegal", str(self.world.illegal_move_reward)),
            ("straight", str(self.world.reward_straight)),
            ("turn_r", str(self.world.reward_turn_right)),
            ("turn_l", str(self.world.reward_turn_left)),
            ("turn_a", str(self.world.reward_turn_around)),
        ]
        grid = tk.Frame(self.sidebar, bg=ui.BG)
        grid.pack(anchor="w", pady=4)
        for key, val in fields:
            row = tk.Frame(grid, bg=ui.BG)
            row.pack(anchor="w")
            ui.label(row, key, muted=True).pack(side=tk.LEFT, padx=(0, 6))
            ent = ui.entry(row, width=8)
            ent.insert(0, val)
            ent.pack(side=tk.LEFT)
            self.param_entries[key] = ent

        ui.button(self.sidebar, "Apply parameters", self._on_apply_params).pack(fill="x", pady=4)
        self.param_message = ui.label(self.sidebar, "", muted=True)
        self.param_message.pack(anchor="w")

    def _on_change_world(self) -> None:
        self.change_world_requested = True
        self.window.destroy()

    def _on_step(self) -> None:
        if self.trainer.training_finished():
            self.status_label.config(text="Status: finished (all episodes)", fg=ui.MUTED)
            return
        self._sync_training_params_from_ui()
        rec = self.trainer.step()
        if rec is None:
            return
        self.trail.append(rec.next_state)
        self._update_labels(rec)
        self._mark_policy_final_if_done()
        self._draw()

    def _on_run_episode(self) -> None:
        if self.trainer.training_finished():
            return
        self._sync_training_params_from_ui()
        self.trainer.start_episode()
        self.trail = [self.trainer.state]
        while self.trainer.can_step():
            rec = self.trainer.step()
            if rec is None:
                break
            self.trail.append(rec.next_state)
            self.window.update_idletasks()
        if self.trainer.last_step:
            self._update_labels(self.trainer.last_step)
        self._mark_policy_final_if_done()
        self._draw()

    def _on_train_all(self) -> None:
        if self.trainer.training_finished():
            return
        self._sync_training_params_from_ui()
        remaining = self.trainer.num_episodes - self.trainer.total_episodes_finished
        self.trainer.run_all_episodes()
        self.trail = []
        self.status_label.config(
            text=f"Training done ({remaining} episodes, like solver.train)",
            fg=ui.ACCENT,
        )
        self._refresh_stats()
        self._update_episode_labels()
        self._mark_policy_final_if_done()
        self._draw()

    def _on_show_policy(self) -> None:
        if not self.trainer.training_finished():
            messagebox.showinfo(
                "Policy not ready",
                "Finish training first (Train all episodes, or complete every episode).",
                parent=self.window,
            )
            return
        report = self.trainer.format_policy_report()
        win = tk.Toplevel(self.window)
        win.title("Learned policy")
        ui.setup(win)
        text = scrolledtext.ScrolledText(
            win,
            width=56,
            height=18,
            font=("Courier", 10),
            bg=ui.BG2,
            fg=ui.FG,
        )
        text.pack(fill="both", expand=True, padx=8, pady=8)
        text.insert("1.0", report)
        text.configure(state="disabled")

    def _on_reset_q(self) -> None:
        self.trainer.reset_training()
        self.policy_final_ready = False
        self.trail = []
        self.status_label.config(text="Q reset — press Next step or Run episode", fg=ui.MUTED)
        self.last_label.config(text="")
        self._refresh_stats()
        self._update_episode_labels()
        self._refresh_policy_panel_state()
        self._draw()

    def _on_apply_params(self) -> None:
        try:
            alpha = float(self.param_entries["alpha"].get())
            gamma = float(self.param_entries["gamma"].get())
            eps0 = float(self.param_entries["epsilon_start"].get())
            eps1 = float(self.param_entries["epsilon_end"].get())
            decay = float(self.param_entries["epsilon_decay"].get())
            max_steps = int(self.param_entries["max_steps"].get())
            num_ep = int(self.param_entries["episodes"].get())
            goal = float(self.param_entries["goal"].get())
            illegal = float(self.param_entries["illegal"].get())
            straight = float(self.param_entries["straight"].get())
            tr = float(self.param_entries["turn_r"].get())
            tl = float(self.param_entries["turn_l"].get())
            ta = float(self.param_entries["turn_a"].get())
        except ValueError:
            self._set_param_message("Invalid number in parameters", error=True)
            return

        if not (0.0 < gamma <= 1.0 and 0.0 <= alpha <= 1.0):
            self._set_param_message("α, γ must be in valid ranges", error=True)
            return

        self.trainer.alpha = alpha
        self.trainer.gamma = gamma
        self.trainer.epsilon_start = eps0
        self.trainer.epsilon_end = eps1
        self.trainer.epsilon_decay = decay
        self.trainer.max_steps = max_steps
        self.trainer.num_episodes = num_ep
        self.world.goal_reward = goal
        self.world.illegal_move_reward = illegal
        self.world.reward_straight = straight
        self.world.reward_turn_right = tr
        self.world.reward_turn_left = tl
        self.world.reward_turn_around = ta
        self.trainer.reset_training()
        self.policy_final_ready = False
        self.trail = []
        self._set_param_message("Applied — Q reset")
        self._update_episode_labels()
        self._refresh_policy_panel_state()
        self._draw()

    def _set_param_message(self, text: str, *, error: bool = False) -> None:
        if self.param_message:
            self.param_message.config(text=text, fg="#f87171" if error else ui.MUTED)

    def _refresh_stats(self) -> None:
        finished = self.trainer.total_episodes_finished
        ok = self.trainer.success_count
        self.stats_label.config(
            text=f"Goals reached: {ok} / {finished} episodes"
            + (
                f"  (last return={self.trainer.last_episode_return:.0f}"
                f"{' ✓' if self.trainer.last_episode_solved else ''})"
                if finished
                else ""
            ),
        )

    def _update_episode_labels(self) -> None:
        done = self.trainer.total_episodes_finished
        total = self.trainer.num_episodes
        if self.trainer.training_finished():
            text = f"Episodes: {done}/{total} — training complete"
        elif not self.trainer.episode_done and self.trainer.episode > 0:
            text = f"Episodes: {done}/{total} done — running #{self.trainer.episode}"
        elif done == 0:
            text = f"Episodes: 0/{total} — not started"
        else:
            text = f"Episodes: {done}/{total} done"
        self.ep_label.config(text=text)

    def _update_labels(self, rec) -> None:
        self._refresh_stats()
        self._update_episode_labels()
        self.step_label.config(text=f"Step: {rec.step}/{self.trainer.max_steps}")
        self.eps_label.config(text=f"ε: {rec.epsilon:.3f}")
        explore = "explore" if rec.explored else "greedy"
        self.last_label.config(
            text=(
                f"{explore}: {ACTION_GLYPHS[rec.action]}  r={rec.reward:.0f}\n"
                f"Q({ACTION_GLYPHS[rec.action]}): {rec.q_before:.2f} → {rec.q_after:.2f}\n"
                f"return={rec.episode_return:.1f}"
            ),
        )
        if rec.done:
            self.status_label.config(text="Status: goal reached", fg=ui.ACCENT)
        elif self.trainer.episode_done:
            self.status_label.config(text="Status: episode ended", fg=ui.FG)
        else:
            self.status_label.config(text="Status: learning", fg=ui.FG)

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

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_q_heatmap()
        self._draw_policy_path()
        self._draw_obstacles()
        self._draw_trail_and_robot()
        self._draw_start_and_goal()

    def _draw_board(self) -> None:
        half = self._half_extent_m
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=ui.BOARD, outline=ui.BORDER, width=1)
        lw = max(1.0, self._meters_to_pixels(LINE_WIDTH_M))
        for center in self._col_line_centers():
            self.canvas.create_line(*self._world_to_canvas(center, half), *self._world_to_canvas(center, -half), fill=ui.LINE, width=lw)
        for center in self._row_line_centers():
            self.canvas.create_line(*self._world_to_canvas(-half, center), *self._world_to_canvas(half, center), fill=ui.LINE, width=lw)

    def _draw_q_heatmap(self) -> None:
        if not self.policy_final_ready:
            return
        view_heading = self._selected_policy_heading()
        box_half_m = self.world.spacing_m * 0.42
        for row in range(self.world.rows):
            for col in range(self.world.cols):
                cell = GridCell(row, col)
                if self.world.is_obstacle(cell):
                    continue
                pose = PoseState(cell, view_heading)
                value = self.trainer.max_q(pose)
                x_m, y_m = self.world.world_xy(cell)
                x0, y0 = self._world_to_canvas(x_m - box_half_m, y_m + box_half_m)
                x1, y1 = self._world_to_canvas(x_m + box_half_m, y_m - box_half_m)
                fill = self._q_to_color(value)
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="", width=0)
                if cell == self.world.goal:
                    continue
                cx, cy = self._world_to_canvas(x_m, y_m)
                text_color = "#333333" if self._is_light(fill) else "#ffffff"
                self.canvas.create_text(
                    cx,
                    cy,
                    text=f"{value:.0f}",
                    fill=text_color,
                    font=(ui.FONT[0], 8, "bold"),
                )
                self._draw_policy_arrow(cx, cy, view_heading, self.trainer.greedy_action(pose))

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

    def _q_to_color(self, value: float) -> str:
        if value > 0:
            return "#dbeafe" if value < 50 else "#93c5fd"
        if value < 0:
            return "#fee2e2"
        return ui.CELL_FREE

    @staticmethod
    def _is_light(hex_color: str) -> bool:
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return 0.299 * r + 0.587 * g + 0.114 * b > 140.0

    def _draw_policy_path(self) -> None:
        if not self.policy_final_ready:
            return
        start = PoseState(self.world.start, self._selected_policy_heading())
        path = self.trainer.greedy_rollout(start)
        if len(path) < 2:
            return
        pts = [self._world_to_canvas(*self.world.world_xy(s.cell)) for s in path]
        flat = [c for p in pts for c in p]
        self.canvas.create_line(*flat, fill=ui.CELL_START, width=3, smooth=False)
        hx, hy = self._world_to_canvas(*self.world.world_xy(start.cell))
        ux, uy = _HEADING_UNIT_VEC[start.heading]
        tick = self._meters_to_pixels(self.world.spacing_m * 0.28)
        self.canvas.create_line(hx, hy, hx + ux * tick, hy - uy * tick, fill=ui.CELL_START, width=3)

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self.world.obstacles:
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - half, y_m + half)
            x1, y1 = self._world_to_canvas(x_m + half, y_m - half)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=ui.OBSTACLE, outline=ui.BORDER, width=1)

    def _draw_trail_and_robot(self) -> None:
        if len(self.trail) >= 2:
            pts = [self._world_to_canvas(*self.world.world_xy(s.cell)) for s in self.trail]
            flat = [c for p in pts for c in p]
            self.canvas.create_line(*flat, fill=ui.TRAIL, width=2, smooth=True)
        state = self.trainer.state
        x_m, y_m = self.world.world_xy(state.cell)
        cx, cy = self._world_to_canvas(x_m, y_m)
        r_px = max(10.0, self._meters_to_pixels(0.055))
        self.canvas.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px, fill=ui.ROBOT, outline="#fff", width=2)
        ux, uy = _HEADING_UNIT_VEC[state.heading]
        tick = self._meters_to_pixels(self.world.spacing_m * 0.22)
        self.canvas.create_line(cx, cy, cx + ux * tick, cy - uy * tick, fill="#fff", width=2)

    def _draw_start_and_goal(self) -> None:
        r_px = max(8.0, self._meters_to_pixels(0.04))
        sx, sy = self._world_to_canvas(*self.world.world_xy(self.world.start))
        self.canvas.create_oval(sx - r_px, sy - r_px, sx + r_px, sy + r_px, outline=ui.CELL_START, width=2)
        gx, gy = self._world_to_canvas(*self.world.world_xy(self.world.goal))
        self.canvas.create_oval(gx - r_px, gy - r_px, gx + r_px, gy + r_px, fill=ui.CELL_GOAL, outline=ui.BORDER)
