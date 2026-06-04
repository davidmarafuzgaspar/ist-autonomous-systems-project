"""Map setup + train + known map with policy (current heading) + run/step/manual."""

from __future__ import annotations

import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox
from typing import Callable

from solver import ACTION_LABELS, ACTION_STRAIGHT
from world import Heading

from .model import AUTO_MISSION_MAX_STEPS, FREE, OBSTACLE, RealRuntimeSim, Scenario

MIN_GRID = 2
MAX_GRID = 12

BG = "#1e1e1e"
BG2 = "#2a2a2a"
FG = "#f0f0f0"
MUTED = "#9ca3af"
ACCENT = "#4a9eff"
BORDER = "#404040"
CANVAS_BG = "#252525"
BOARD = "#fafafa"
LINE = "#d4d4d4"
CELL_OBS = "#6b7280"
CELL_GOAL = "#4a9eff"
CELL_START = "#22c55e"
POLICY = "#4a9eff"
ROBOT = "#e91e63"
PATH_OUTLINE = "#22c55e"
TRAIL = "#ff80ab"
FONT = ("Ubuntu", 10)
FONT_BOLD = ("Ubuntu", 11, "bold")
FONT_TITLE = ("Ubuntu", 14, "bold")

MODE_OBS = "obstacle"
MODE_START = "start"
MODE_GOAL = "goal"

_HEADING_VEC = {"N": (0.0, -1.0), "E": (1.0, 0.0), "S": (0.0, 1.0), "W": (-1.0, -1.0)}
_DR_DC = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}


def _setup_fonts(root: tk.Misc) -> None:
    global FONT, FONT_BOLD, FONT_TITLE
    root.configure(bg=BG)
    try:
        for family in ("Segoe UI", "Ubuntu", "Helvetica Neue", "Arial"):
            if family in tkfont.families(root):
                FONT = (family, 10)
                FONT_BOLD = (family, 11, "bold")
                FONT_TITLE = (family, 14, "bold")
                break
    except tk.TclError:
        pass


def _label(parent: tk.Misc, text: str, *, muted: bool = False) -> tk.Label:
    return tk.Label(
        parent, text=text, font=FONT, fg=MUTED if muted else FG,
        bg=parent.cget("bg") or BG, anchor="w",
    )


def _button(parent: tk.Misc, text: str, command: Callable[[], None], *, primary: bool = False) -> tk.Button:
    return tk.Button(
        parent, text=text, command=command, font=FONT_BOLD if primary else FONT,
        bg=ACCENT if primary else BG2, fg="#fff" if primary else FG,
        relief="flat", padx=8, pady=4, cursor="hand2",
    )


def _entry(parent: tk.Misc, width: int = 4) -> tk.Entry:
    return tk.Entry(
        parent, width=width, font=FONT, bg=BG2, fg=FG, insertbackground=FG,
        relief="flat", highlightthickness=1, highlightbackground=BORDER, highlightcolor=ACCENT,
    )


def _geom(size_px: int, rows: int, cols: int) -> tuple[float, float, float]:
    pad = 16
    cell = (size_px - 2 * pad) / max(rows, cols)
    x0 = (size_px - cell * cols) / 2
    y0 = (size_px - cell * rows) / 2
    return x0, y0, cell


def _cell_center(size_px: int, rows: int, cols: int, row: int, col: int) -> tuple[float, float]:
    x0, y0, cell = _geom(size_px, rows, cols)
    return x0 + (col + 0.5) * cell, y0 + (row + 0.5) * cell


def _draw_grid(
    canvas: tk.Canvas,
    *,
    rows: int,
    cols: int,
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    robot: tuple[int, int, str] | None,
    trail: list[tuple[int, int, str]],
    policy_fn: Callable[[int, int, Heading], int | None] | None,
    policy_heading: Heading,
    path_cells: set[tuple[int, int]],
    size_px: int,
    show_robot: bool = True,
    show_trail: bool = False,
    show_path: bool = False,
    mark_start_goal: bool = False,
) -> None:
    canvas.delete("all")
    x0, y0, cell = _geom(size_px, rows, cols)

    canvas.create_rectangle(
        x0, y0, x0 + cell * cols, y0 + cell * rows,
        fill=BOARD, outline=BORDER,
    )
    lw = max(1, int(cell * 0.12))
    for c in range(cols):
        px = x0 + (c + 0.5) * cell
        canvas.create_line(px, y0, px, y0 + cell * rows, fill=LINE, width=lw)
    for r in range(rows):
        py = y0 + (r + 0.5) * cell
        canvas.create_line(x0, py, x0 + cell * cols, py, fill=LINE, width=lw)

    if show_trail and len(trail) >= 2:
        pts: list[float] = []
        for r, c, _ in trail:
            px, py = _cell_center(size_px, rows, cols, r, c)
            pts.extend([px, py])
        canvas.create_line(*pts, fill=TRAIL, width=3, smooth=True)

    for row in range(rows):
        for col in range(cols):
            x1, y1 = x0 + col * cell, y0 + row * cell
            x2, y2 = x1 + cell, y1 + cell
            is_robot = show_robot and robot and (row, col) == (robot[0], robot[1])
            is_goal = (row, col) == goal
            is_start = (row, col) == start
            is_obs = grid[row][col] == OBSTACLE

            if is_robot:
                fill = ROBOT
            elif is_goal and mark_start_goal:
                fill = CELL_GOAL
            elif is_start and mark_start_goal and not is_robot:
                fill = CELL_START
            elif is_obs:
                fill = CELL_OBS
            else:
                fill = ""
            if fill:
                canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="")

            if show_path and (row, col) in path_cells and not is_goal and not is_obs:
                canvas.create_rectangle(x1, y1, x2, y2, outline=PATH_OUTLINE, width=2)

            if is_robot:
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                canvas.create_text(cx, cy - 2, text="R", fill="#fff", font=FONT_BOLD)
                ux, uy = _HEADING_VEC.get(robot[2], (0.0, 1.0))
                canvas.create_line(
                    cx, cy, cx + ux * cell * 0.28, cy + uy * cell * 0.28,
                    fill="#fff", width=2,
                )
            elif mark_start_goal and is_goal:
                canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text="G", fill="#fff", font=FONT_BOLD)
            elif mark_start_goal and is_start:
                canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text="S", fill="#fff", font=FONT_BOLD)
            elif is_goal and not mark_start_goal:
                canvas.create_rectangle(x1, y1, x2, y2, fill=CELL_GOAL, outline="")
                canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text="G", fill="#fff", font=FONT_BOLD)

            if policy_fn and not is_obs and not is_goal:
                act = policy_fn(row, col, policy_heading)
                if act is not None:
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2 + cell * 0.12
                    if act == ACTION_STRAIGHT:
                        mh = policy_heading
                    elif act == 1:
                        mh = policy_heading.turn_right()
                    elif act == 2:
                        mh = policy_heading.turn_left()
                    else:
                        mh = policy_heading.turn_right().turn_right()
                    dr, dc = _DR_DC[mh]
                    canvas.create_line(
                        cx, cy, cx + dc * cell * 0.2, cy + dr * cell * 0.2,
                        fill=POLICY, width=2, arrow=tk.LAST, arrowshape=(5, 6, 3),
                    )


class MapSetupDialog:
    def __init__(self, *, initial: Scenario | None = None) -> None:
        self._result: Scenario | None = None
        if initial is not None:
            self._rows, self._cols = initial.rows, initial.cols
            self._grid = [row[:] for row in initial.grid]
            self._start, self._goal = initial.start, initial.goal
            self._heading = initial.start_heading.name
        else:
            self._rows, self._cols = 5, 5
            self._grid = [[FREE] * 5 for _ in range(5)]
            self._start, self._goal = (0, 0), (4, 4)
            self._heading = "S"
        self._canvas_px = 500

        self.root = tk.Tk()
        self.root.title("Map setup")
        _setup_fonts(self.root)
        self.mode_var = tk.StringVar(master=self.root, value=MODE_OBS)
        self.heading_var = tk.StringVar(master=self.root, value=self._heading)

        root = tk.Frame(self.root, bg=BG, padx=16, pady=16)
        root.pack()
        _label(root, "Map setup", muted=False).configure(font=FONT_TITLE)
        _label(root, "Rows/cols, obstacles, start, goal, heading. Then Continue.", muted=True).pack(
            anchor="w", pady=(2, 10),
        )

        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", pady=(0, 8))
        _label(top, "Rows").pack(side=tk.LEFT)
        self.rows_entry = _entry(top, 4)
        self.rows_entry.insert(0, str(self._rows))
        self.rows_entry.pack(side=tk.LEFT, padx=(4, 12))
        _label(top, "Cols").pack(side=tk.LEFT)
        self.cols_entry = _entry(top, 4)
        self.cols_entry.insert(0, str(self._cols))
        self.cols_entry.pack(side=tk.LEFT, padx=(4, 12))
        _button(top, "Apply size", self._apply_size).pack(side=tk.LEFT)

        modes = tk.Frame(root, bg=BG)
        modes.pack(anchor="w", pady=4)
        for text, val in (("Obstacle", MODE_OBS), ("Start", MODE_START), ("Goal", MODE_GOAL)):
            tk.Radiobutton(
                modes, text=text, variable=self.mode_var, value=val,
                bg=BG, fg=FG, selectcolor=BG2, activebackground=BG,
            ).pack(side=tk.LEFT, padx=(0, 10))

        head = tk.Frame(root, bg=BG)
        head.pack(anchor="w", pady=4)
        _label(head, "Start heading:").pack(side=tk.LEFT, padx=(0, 6))
        for h in ("N", "E", "S", "W"):
            tk.Radiobutton(
                head, text=h, variable=self.heading_var, value=h,
                command=self._redraw, bg=BG, fg=FG, selectcolor=BG2,
            ).pack(side=tk.LEFT, padx=2)

        self.canvas = tk.Canvas(root, width=self._canvas_px, height=self._canvas_px, bg=CANVAS_BG, highlightthickness=0)
        self.canvas.pack(pady=8)
        self.canvas.bind("<Button-1>", self._on_click)

        btns = tk.Frame(root, bg=BG)
        btns.pack(fill="x")
        _button(btns, "Clear obstacles", self._clear_obstacles).pack(side=tk.LEFT)
        _button(btns, "Continue", self._confirm, primary=True).pack(side=tk.RIGHT)
        _button(btns, "Cancel", self.root.destroy).pack(side=tk.RIGHT, padx=6)
        self._redraw()

    def run(self) -> Scenario | None:
        self.root.mainloop()
        return self._result

    def _apply_size(self) -> None:
        try:
            r, c = int(self.rows_entry.get()), int(self.cols_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Rows/cols must be integers.")
            return
        if not (MIN_GRID <= r <= MAX_GRID and MIN_GRID <= c <= MAX_GRID):
            messagebox.showerror("Error", f"Size {MIN_GRID}–{MAX_GRID}.")
            return
        self._rows, self._cols = r, c
        self._grid = [[FREE] * c for _ in range(r)]
        self._start, self._goal = (0, 0), (r - 1, c - 1)
        self._redraw()

    def _clear_obstacles(self) -> None:
        for r in range(self._rows):
            for c in range(self._cols):
                if (r, c) not in (self._start, self._goal):
                    self._grid[r][c] = FREE
        self._redraw()

    def _cell_at(self, event: tk.Event) -> tuple[int, int] | None:
        x0, y0, cell_px = _geom(self._canvas_px, self._rows, self._cols)
        col = int((event.x - x0) / cell_px)
        row = int((event.y - y0) / cell_px)
        if 0 <= row < self._rows and 0 <= col < self._cols:
            return row, col
        return None

    def _on_click(self, event: tk.Event) -> None:
        cell = self._cell_at(event)
        if cell is None:
            return
        r, c = cell
        mode = self.mode_var.get()
        if mode == MODE_START and self._grid[r][c] != OBSTACLE:
            self._start = (r, c)
        elif mode == MODE_GOAL and self._grid[r][c] != OBSTACLE and (r, c) != self._start:
            self._goal = (r, c)
        elif mode == MODE_OBS and (r, c) not in (self._start, self._goal):
            self._grid[r][c] = OBSTACLE if self._grid[r][c] != OBSTACLE else FREE
        self._redraw()

    def _confirm(self) -> None:
        if self._start == self._goal:
            messagebox.showerror("Error", "Start ≠ goal.")
            return
        self._result = Scenario(
            grid=[row[:] for row in self._grid],
            start=self._start,
            goal=self._goal,
            start_heading=Heading.from_str(self.heading_var.get()),
        )
        self.root.destroy()

    def _redraw(self) -> None:
        _draw_grid(
            self.canvas,
            rows=self._rows,
            cols=self._cols,
            grid=self._grid,
            start=self._start,
            goal=self._goal,
            robot=(self._start[0], self._start[1], self.heading_var.get()),
            trail=[],
            policy_fn=None,
            policy_heading=Heading.S,
            path_cells=set(),
            size_px=self._canvas_px,
            mark_start_goal=True,
        )


def run_map_setup(*, initial: Scenario | None = None) -> Scenario | None:
    return MapSetupDialog(initial=initial).run()


class RealRuntimeViewer:
    def __init__(self, sim: RealRuntimeSim) -> None:
        self.sim = sim
        self.change_world_requested = False
        self._main_px = 600
        self._policy_px = 300
        self._sidebar_px = 340

        self.window = tk.Tk()
        self.window.title("Policy runtime")
        _setup_fonts(self.window)
        self.window.minsize(1000, 820)
        self.window.geometry("1040x860")
        self.policy_heading_var = tk.StringVar(
            master=self.window, value=sim.robot.heading.name,
        )

        body = tk.Frame(self.window, bg=BG)
        body.pack(fill="both", expand=True, padx=10, pady=10)

        left = tk.Frame(body, bg=BG)
        left.pack(side=tk.LEFT, fill="both", expand=True)
        _label(left, "Execution — robot, trail, greedy path (atual pose)", muted=True).pack(anchor="w")
        self.main_canvas = tk.Canvas(
            left, width=self._main_px, height=self._main_px,
            bg=CANVAS_BG, highlightthickness=0,
        )
        self.main_canvas.pack()
        self.main_canvas.bind("<Button-1>", self._on_main_click)
        self.path_label = _label(left, "", muted=True)
        self.path_label.pack(anchor="w", pady=(4, 0))

        right = tk.Frame(body, bg=BG, width=self._sidebar_px)
        right.pack(side=tk.RIGHT, fill="y", padx=(10, 0))
        right.pack_propagate(False)

        _label(right, "Known map — π only (heading to inspect)", muted=True).pack(anchor="w")
        self.policy_canvas = tk.Canvas(
            right, width=self._policy_px, height=self._policy_px,
            bg=CANVAS_BG, highlightthickness=0,
        )
        self.policy_canvas.pack(pady=(0, 6))

        _label(right, "Policy view heading:", muted=True).pack(anchor="w")
        ph = tk.Frame(right, bg=BG)
        ph.pack(anchor="w")
        for h in ("N", "E", "S", "W"):
            tk.Radiobutton(
                ph, text=h, variable=self.policy_heading_var, value=h,
                command=self._redraw, bg=BG, fg=FG, selectcolor=BG2,
            ).pack(side=tk.LEFT, padx=3)
        self.status = _label(right, "1) Train  2) Run / step / manual")
        self.status.pack(anchor="w", pady=(0, 8))

        _button(right, "Train", self._on_train, primary=True).pack(fill="x", pady=2)

        _label(right, "Execute", muted=True).pack(anchor="w", pady=(8, 0))
        _button(right, "Auto run (start → goal)", self._on_auto_run, primary=True).pack(fill="x", pady=2)
        _button(right, "Back to start", self._on_reset).pack(fill="x", pady=2)

        _label(right, "Step (one π action: turn or forward)", muted=True).pack(anchor="w", pady=(8, 0))
        _button(right, "Next step", self._on_next_step).pack(fill="x", pady=2)

        _label(right, "Manual (left map)", muted=True).pack(anchor="w", pady=(8, 0))
        tr = tk.Frame(right, bg=BG)
        tr.pack(anchor="w")
        for h in ("N", "E", "S", "W"):
            _button(tr, h, lambda hd=h: self._manual_heading(hd)).pack(side=tk.LEFT, padx=2)
        _label(right, "Or click adjacent cell on execution map", muted=True).pack(anchor="w")

        _button(right, "Change world", self._on_change_world).pack(fill="x", pady=(12, 0))

        self._redraw()

    def run(self) -> bool:
        self.change_world_requested = False
        self.window.mainloop()
        return self.change_world_requested

    def _policy_fn(self, row: int, col: int, heading: Heading) -> int | None:
        return self.sim.policy_action(row, col, heading)

    def _path_cells(self) -> set[tuple[int, int]]:
        if not self.sim.trained:
            return set()
        path = self.sim.optimal_path_from_robot()
        return {(r, c) for r, c, _ in path}

    def _redraw(self) -> None:
        ph = Heading.from_str(self.policy_heading_var.get())
        robot = (self.sim.robot.row, self.sim.robot.col, self.sim.robot.heading.name)
        path_cells = self._path_cells()

        start = self.sim.scenario.start
        goal = self.sim.scenario.goal

        _draw_grid(
            self.main_canvas,
            rows=self.sim.rows,
            cols=self.sim.cols,
            grid=self.sim.scenario.grid,
            start=start,
            goal=goal,
            robot=robot,
            trail=self.sim.trail,
            policy_fn=None,
            policy_heading=ph,
            path_cells=path_cells,
            size_px=self._main_px,
            show_robot=True,
            show_trail=True,
            show_path=True,
            mark_start_goal=False,
        )

        _draw_grid(
            self.policy_canvas,
            rows=self.sim.rows,
            cols=self.sim.cols,
            grid=self.sim.scenario.grid,
            start=start,
            goal=goal,
            robot=None,
            trail=[],
            policy_fn=self._policy_fn if self.sim.trained else None,
            policy_heading=ph,
            path_cells=set(),
            size_px=self._policy_px,
            show_robot=False,
            show_trail=False,
            show_path=False,
            mark_start_goal=True,
        )

        if self.sim.trained:
            path = self.sim.optimal_path_from(
                self.sim.robot.row, self.sim.robot.col, self.sim.robot.heading,
            )
            if len(path) <= 1:
                self.path_label.config(text="Greedy path from robot: —")
            elif path[-1][:2] == self.sim.scenario.goal:
                self.path_label.config(text=f"Greedy path from robot: {len(path) - 1} step(s) to goal")
            else:
                self.path_label.config(text=f"Greedy path from robot: {len(path) - 1} step(s) (stops)")
            act = self.sim.current_policy_action()
            if act is not None:
                self.status.config(
                    text=f"{self.sim.robot.pose_str()} · here π={ACTION_LABELS[act]}",
                )
        else:
            self.path_label.config(text="Greedy path: train first.")

    def _on_train(self) -> None:
        self.sim.train()
        self.status.config(text="Trained. Auto run, Next step, or manual on left map.")
        self._redraw()

    def _on_auto_run(self) -> None:
        if not self.sim.trained:
            self.sim.train()
        self.sim.reset_to_start()
        for _ in range(AUTO_MISSION_MAX_STEPS):
            msg, done = self.sim.execute_policy_cycle()
            self._redraw()
            self.window.update_idletasks()
            time.sleep(0.1)
            if done:
                self.status.config(text=f"Done: {msg}")
                break
            if "Train" in msg:
                break
        self._redraw()

    def _on_next_step(self) -> None:
        if not self.sim.trained:
            self.status.config(text="Train first.")
            return
        msg, done = self.sim.execute_policy_cycle()
        self.status.config(text=msg if not done else f"Done: {msg}")
        self._redraw()

    def _manual_heading(self, name: str) -> None:
        self.sim.set_heading(Heading.from_str(name))
        self._redraw()

    def _on_main_click(self, event: tk.Event) -> None:
        if not self.sim.trained:
            return
        x0, y0, cell_px = _geom(self._main_px, self.sim.rows, self.sim.cols)
        col = int((event.x - x0) / cell_px)
        row = int((event.y - y0) / cell_px)
        if 0 <= row < self.sim.rows and 0 <= col < self.sim.cols:
            self.status.config(text=self.sim.manual_goto_adjacent(row, col))
            self._redraw()

    def _on_reset(self) -> None:
        self.sim.reset_to_start()
        self.status.config(text="Back to start.")
        self._redraw()

    def _on_change_world(self) -> None:
        self.change_world_requested = True
        self.window.destroy()
