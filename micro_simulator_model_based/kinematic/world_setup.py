from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox

from . import ui_theme as ui

from .board import CrossBoard, Point2D
from .config import (
    DEFAULT_COLUMNS,
    DEFAULT_LINES,
    MAX_COLUMNS,
    MAX_LINES,
    MIN_COLUMNS,
    MIN_LINES,
    BoardConfig,
)
from .obstacles import RectangleObstacle, obstacle_at_crossing, snap_obstacles_to_board

MODE_PLACE = "place"
MODE_ERASE = "erase"


@dataclass
class WorldSetup:
    lines: int
    columns: int
    obstacles: list[RectangleObstacle]


def default_world_setup() -> WorldSetup:
    return WorldSetup(lines=DEFAULT_LINES, columns=DEFAULT_COLUMNS, obstacles=[])


def run_world_setup(*, initial: WorldSetup | None = None) -> WorldSetup | None:
    return WorldSetupDialog(initial=initial).run()


class WorldSetupDialog:
    def __init__(self, *, initial: WorldSetup | None = None) -> None:
        seed = initial or default_world_setup()
        self._lines = seed.lines
        self._columns = seed.columns
        self._board = CrossBoard(config=BoardConfig(lines=self._lines, columns=self._columns))
        self._obstacles = snap_obstacles_to_board(list(seed.obstacles), self._board)
        self._result: WorldSetup | None = None
        self._canvas_px = 520

        self.root = tk.Tk()
        self.root.title("World setup")
        ui.setup(self.root)

        root = tk.Frame(self.root, bg=ui.BG, padx=16, pady=16)
        root.pack()

        title = ui.label(root, "World setup")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w")
        ui.label(root, "Lines × columns. Click a junction to toggle an obstacle.", muted=True).pack(
            anchor="w", pady=(2, 12)
        )

        top = tk.Frame(root, bg=ui.BG)
        top.pack(fill="x", pady=(0, 10))
        ui.label(top, "Lines").pack(side=tk.LEFT)
        self.lines_entry = ui.entry(top, width=4)
        self.lines_entry.insert(0, str(self._lines))
        self.lines_entry.pack(side=tk.LEFT, padx=(4, 12))
        ui.label(top, "Columns").pack(side=tk.LEFT)
        self.columns_entry = ui.entry(top, width=4)
        self.columns_entry.insert(0, str(self._columns))
        self.columns_entry.pack(side=tk.LEFT, padx=(4, 12))
        ui.button(top, "Apply grid", self._apply_grid).pack(side=tk.LEFT)

        self.mode_var = tk.StringVar(master=self.root, value=MODE_PLACE)
        modes = tk.Frame(root, bg=ui.BG)
        modes.pack(anchor="w", pady=(0, 8))
        for text, val in (("Place obstacle", MODE_PLACE), ("Erase obstacle", MODE_ERASE)):
            rb = ui.radio(modes, text, self.mode_var, val)
            rb.configure(command=self._redraw)
            rb.pack(side=tk.LEFT, padx=(0, 12))

        self.canvas = tk.Canvas(
            root,
            width=self._canvas_px,
            height=self._canvas_px,
            bg=ui.CANVAS_BG,
            highlightthickness=0,
        )
        self.canvas.pack(pady=(0, 12))
        self.canvas.bind("<Button-1>", self._on_click)

        btns = tk.Frame(root, bg=ui.BG)
        btns.pack(fill="x")
        ui.button(btns, "Clear obstacles", self._clear_obstacles).pack(side=tk.LEFT)
        ui.button(btns, "Continue", self._on_confirm, primary=True).pack(side=tk.RIGHT)
        ui.button(btns, "Cancel", self.root.destroy).pack(side=tk.RIGHT, padx=(0, 8))

        self._redraw()

    def run(self) -> WorldSetup | None:
        self.root.mainloop()
        return self._result

    def _parse_grid(self) -> tuple[int, int] | None:
        try:
            lines = int(self.lines_entry.get())
            columns = int(self.columns_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Lines and columns must be integers.")
            return None
        if not (MIN_LINES <= lines <= MAX_LINES and MIN_COLUMNS <= columns <= MAX_COLUMNS):
            messagebox.showerror(
                "Error",
                f"Lines must be {MIN_LINES}–{MAX_LINES}, columns {MIN_COLUMNS}–{MAX_COLUMNS}.",
            )
            return None
        return lines, columns

    def _apply_grid(self) -> None:
        parsed = self._parse_grid()
        if parsed is None:
            return
        self._lines, self._columns = parsed
        self._board = CrossBoard(config=BoardConfig(lines=self._lines, columns=self._columns))
        self._obstacles = snap_obstacles_to_board(self._obstacles, self._board)
        self._redraw()

    def _clear_obstacles(self) -> None:
        self._obstacles.clear()
        self._redraw()

    def _on_confirm(self) -> None:
        self._result = WorldSetup(
            lines=self._lines,
            columns=self._columns,
            obstacles=snap_obstacles_to_board(self._obstacles, self._board),
        )
        self.root.destroy()

    def _obstacle_at_junction(self, junction: Point2D) -> RectangleObstacle | None:
        key = self._board.crossing_key(junction)
        for obstacle in self._obstacles:
            if self._board.crossing_key(Point2D(obstacle.center_x_m, obstacle.center_y_m)) == key:
                return obstacle
        return None

    def _on_click(self, event: tk.Event) -> None:
        x_m, y_m = self._board.canvas_to_world(event.x, event.y, self._canvas_px)
        junction = self._board.nearest_crossing(x_m, y_m)
        if junction is None:
            messagebox.showinfo("Junction", "Click closer to a line crossing.")
            return

        existing = self._obstacle_at_junction(junction)

        if self.mode_var.get() == MODE_ERASE:
            if existing is None:
                messagebox.showinfo("Erase", "No obstacle at this junction.")
            else:
                self._obstacles.remove(existing)
            self._redraw()
            return

        if existing is not None:
            self._obstacles.remove(existing)
            self._redraw()
            return

        name = f"obs_{len(self._obstacles)}"
        self._obstacles.append(obstacle_at_crossing(name, junction))
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.delete("all")
        cfg = self._board.config
        hx, hy = cfg.half_extent_x_m, cfg.half_extent_y_m
        x0, y0 = self._board.world_to_canvas(-hx, hy, self._canvas_px)
        x1, y1 = self._board.world_to_canvas(hx, -hy, self._canvas_px)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=ui.BOARD, outline="#888", width=2)

        line_width = max(1, int(self._board.meters_to_pixels(cfg.line_width_m, self._canvas_px)))
        extent_y = cfg.line_extent_y_m
        extent_x = cfg.line_extent_x_m
        for center in self._board.line_centers_x():
            xa, ya = self._board.world_to_canvas(center, extent_y, self._canvas_px)
            xb, yb = self._board.world_to_canvas(center, -extent_y, self._canvas_px)
            self.canvas.create_line(xa, ya, xb, yb, fill="black", width=line_width)

        for center in self._board.line_centers_y():
            xa, ya = self._board.world_to_canvas(-extent_x, center, self._canvas_px)
            xb, yb = self._board.world_to_canvas(extent_x, center, self._canvas_px)
            self.canvas.create_line(xa, ya, xb, yb, fill="black", width=line_width)

        slot_r = max(4.0, self._board.meters_to_pixels(cfg.spacing_m * 0.12, self._canvas_px))
        for junction in self._board.crossing_points():
            if self._obstacle_at_junction(junction) is not None:
                continue
            px, py = self._board.world_to_canvas(junction.x, junction.y, self._canvas_px)
            self.canvas.create_oval(
                px - slot_r,
                py - slot_r,
                px + slot_r,
                py + slot_r,
                fill="#e8e8e8",
                outline="#9ca3af",
                width=1,
            )

        for obstacle in self._obstacles:
            x0, y0 = self._board.world_to_canvas(obstacle.min_x_m, obstacle.max_y_m, self._canvas_px)
            x1, y1 = self._board.world_to_canvas(obstacle.max_x_m, obstacle.min_y_m, self._canvas_px)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill="#c96c28", outline="#7a3d14", width=2)
