"""Tk map editor (grid, obstacles, start, goal, heading)."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox

from . import ui_theme as ui
from .world import (
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
            ui.radio(head, h, self.heading_var, h).pack(side=tk.LEFT, padx=(0, 6))

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
                if mark:
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=mark, fill="#fff", font=ui.FONT_BOLD)


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
