"""Tkinter UI: world setup and simulation viewer."""

from __future__ import annotations

import math
import sys
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox
from typing import Callable

from .model import (
    DEFAULT_COLUMNS,
    DEFAULT_LINES,
    MAX_COLUMNS,
    MAX_LINES,
    MIN_COLUMNS,
    MIN_LINES,
    AlphaBotSimulation,
    BoardConfig,
    CrossBoard,
    Point2D,
    RectangleObstacle,
    SensorSnapshot,
    WorldSetup,
    default_world_setup,
    obstacle_at_crossing,
    snap_obstacles_to_board,
)

# --- theme ---
BG = "#1e1e1e"
BG2 = "#2a2a2a"
FG = "#f0f0f0"
MUTED = "#9ca3af"
ACCENT = "#4a9eff"
BORDER = "#404040"
CANVAS_BG = "#252525"
BOARD = "#fafafa"
FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 11, "bold")
FONT_TITLE = ("Segoe UI", 14, "bold")

_MODE_PLACE = "place"
_MODE_ERASE = "erase"


def _setup_theme(root: tk.Misc) -> None:
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


def _label(parent: tk.Misc, text: str, *, muted: bool = False) -> tk.Label:
    return tk.Label(
        parent,
        text=text,
        font=FONT,
        fg=MUTED if muted else FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        anchor="w",
    )


def _button(parent: tk.Misc, text: str, command: Callable[[], None], *, primary: bool = False) -> tk.Button:
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


def _entry(parent: tk.Misc, *, width: int = 6) -> tk.Entry:
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


def _radio(parent: tk.Misc, text: str, variable: tk.StringVar, value: str) -> tk.Radiobutton:
    return tk.Radiobutton(
        parent,
        text=text,
        variable=variable,
        value=value,
        font=FONT,
        fg=FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        selectcolor=BG2,
        activebackground=BG,
        activeforeground=FG,
        anchor="w",
    )


def _hline(parent: tk.Misc) -> None:
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", pady=10)


def run_world_setup(*, initial: WorldSetup | None = None) -> WorldSetup | None:
    return _WorldSetupDialog(initial=initial).run()


class _WorldSetupDialog:
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
        _setup_theme(self.root)

        root = tk.Frame(self.root, bg=BG, padx=16, pady=16)
        root.pack()

        title = _label(root, "World setup")
        title.configure(font=FONT_TITLE)
        title.pack(anchor="w")
        _label(root, "Lines × columns. Click a junction to toggle an obstacle.", muted=True).pack(
            anchor="w", pady=(2, 12)
        )

        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", pady=(0, 10))
        _label(top, "Lines").pack(side=tk.LEFT)
        self.lines_entry = _entry(top, width=4)
        self.lines_entry.insert(0, str(self._lines))
        self.lines_entry.pack(side=tk.LEFT, padx=(4, 12))
        _label(top, "Columns").pack(side=tk.LEFT)
        self.columns_entry = _entry(top, width=4)
        self.columns_entry.insert(0, str(self._columns))
        self.columns_entry.pack(side=tk.LEFT, padx=(4, 12))
        _button(top, "Apply grid", self._apply_grid).pack(side=tk.LEFT)

        self.mode_var = tk.StringVar(master=self.root, value=_MODE_PLACE)
        modes = tk.Frame(root, bg=BG)
        modes.pack(anchor="w", pady=(0, 8))
        for text, val in (("Place obstacle", _MODE_PLACE), ("Erase obstacle", _MODE_ERASE)):
            rb = _radio(modes, text, self.mode_var, val)
            rb.configure(command=self._redraw)
            rb.pack(side=tk.LEFT, padx=(0, 12))

        self.canvas = tk.Canvas(
            root,
            width=self._canvas_px,
            height=self._canvas_px,
            bg=CANVAS_BG,
            highlightthickness=0,
        )
        self.canvas.pack(pady=(0, 12))
        self.canvas.bind("<Button-1>", self._on_click)

        btns = tk.Frame(root, bg=BG)
        btns.pack(fill="x")
        _button(btns, "Clear obstacles", self._clear_obstacles).pack(side=tk.LEFT)
        _button(btns, "Continue", self._on_confirm, primary=True).pack(side=tk.RIGHT)
        _button(btns, "Cancel", self.root.destroy).pack(side=tk.RIGHT, padx=(0, 8))
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
        if self.mode_var.get() == _MODE_ERASE:
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
        self._obstacles.append(obstacle_at_crossing(f"obs_{len(self._obstacles)}", junction))
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.delete("all")
        cfg = self._board.config
        hx, hy = cfg.half_extent_x_m, cfg.half_extent_y_m
        x0, y0 = self._board.world_to_canvas(-hx, hy, self._canvas_px)
        x1, y1 = self._board.world_to_canvas(hx, -hy, self._canvas_px)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=BOARD, outline="#888", width=2)
        line_width = max(1, int(self._board.meters_to_pixels(cfg.line_width_m, self._canvas_px)))
        for center in self._board.line_centers_x():
            xa, ya = self._board.world_to_canvas(center, cfg.line_extent_y_m, self._canvas_px)
            xb, yb = self._board.world_to_canvas(center, -cfg.line_extent_y_m, self._canvas_px)
            self.canvas.create_line(xa, ya, xb, yb, fill="black", width=line_width)
        for center in self._board.line_centers_y():
            xa, ya = self._board.world_to_canvas(-cfg.line_extent_x_m, center, self._canvas_px)
            xb, yb = self._board.world_to_canvas(cfg.line_extent_x_m, center, self._canvas_px)
            self.canvas.create_line(xa, ya, xb, yb, fill="black", width=line_width)
        slot_r = max(4.0, self._board.meters_to_pixels(cfg.spacing_m * 0.12, self._canvas_px))
        for junction in self._board.crossing_points():
            if self._obstacle_at_junction(junction) is not None:
                continue
            px, py = self._board.world_to_canvas(junction.x, junction.y, self._canvas_px)
            self.canvas.create_oval(px - slot_r, py - slot_r, px + slot_r, py + slot_r, fill="#e8e8e8", outline="#9ca3af", width=1)
        for obstacle in self._obstacles:
            x0, y0 = self._board.world_to_canvas(obstacle.min_x_m, obstacle.max_y_m, self._canvas_px)
            x1, y1 = self._board.world_to_canvas(obstacle.max_x_m, obstacle.min_y_m, self._canvas_px)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill="#c96c28", outline="#7a3d14", width=2)


class SimulationViewer:
    def __init__(self, simulation: AlphaBotSimulation, dt_s: float = 0.05) -> None:
        self.simulation = simulation
        self.dt_s = dt_s
        self.change_world_requested = False
        self._tick_job: str | None = None

        self.window = tk.Tk()
        self.window.title("AlphaBot2 Simulator")
        _setup_theme(self.window)

        body = tk.Frame(self.window, bg=BG)
        body.pack(fill="both", expand=True)

        self.canvas_size_px = 720
        self.canvas = tk.Canvas(
            body,
            width=self.canvas_size_px,
            height=self.canvas_size_px,
            bg="#f0f0f0",
            highlightthickness=0,
        )
        self.canvas.pack(side=tk.LEFT, padx=(12, 0), pady=12)

        self.sidebar = tk.Frame(body, bg=BG, width=260)
        self.sidebar.pack(side=tk.RIGHT, fill="y", padx=12, pady=12)
        self.sidebar.pack_propagate(False)
        self._build_sidebar()

        self.keys_pressed: set[str] = set()
        self.window.bind("<KeyPress>", self._on_key_press)
        self.window.bind("<KeyRelease>", self._on_key_release)

    def _build_sidebar(self) -> None:
        title = _label(self.sidebar, "AlphaBot2")
        title.configure(font=FONT_TITLE)
        title.pack(anchor="w", pady=(0, 8))
        cfg = self.simulation.board.config
        _label(self.sidebar, f"Grid: {cfg.lines}×{cfg.columns}", muted=True).pack(anchor="w", pady=(0, 4))
        _label(self.sidebar, f"Obstacles: {len(self.simulation.obstacles)}", muted=True).pack(anchor="w", pady=(0, 12))
        _label(self.sidebar, "Controls", muted=True).pack(anchor="w")
        _label(self.sidebar, "WASD / arrows - drive", muted=True).pack(anchor="w")
        _label(self.sidebar, "R - reset pose", muted=True).pack(anchor="w", pady=(0, 16))
        self.line_label = _label(self.sidebar, "Line: —")
        self.line_label.pack(anchor="w")
        self.obstacle_label = _label(self.sidebar, "IR: —")
        self.obstacle_label.pack(anchor="w", pady=(0, 16))
        _hline(self.sidebar)
        _button(self.sidebar, "Change world", self._on_change_world).pack(fill="x", pady=(12, 0))

    def run(self) -> bool:
        self.change_world_requested = False
        self._tick()
        self.window.mainloop()
        return self.change_world_requested

    def _on_change_world(self) -> None:
        self.change_world_requested = True
        if self._tick_job is not None:
            self.window.after_cancel(self._tick_job)
        self.window.destroy()

    def _on_key_press(self, event: tk.Event) -> None:
        if event.keysym.lower() == "r":
            self.simulation.reset()
        elif event.keysym == "space":
            self.simulation.set_command(0.0, 0.0)
        self.keys_pressed.add(event.keysym.lower())

    def _on_key_release(self, event: tk.Event) -> None:
        self.keys_pressed.discard(event.keysym.lower())

    def _tick(self) -> None:
        linear = angular = 0.0
        if "w" in self.keys_pressed or "up" in self.keys_pressed:
            linear += 0.18
        if "s" in self.keys_pressed or "down" in self.keys_pressed:
            linear -= 0.12
        if "a" in self.keys_pressed or "left" in self.keys_pressed:
            angular += 1.4
        if "d" in self.keys_pressed or "right" in self.keys_pressed:
            angular -= 1.4
        self.simulation.set_command(linear_m_s=linear, angular_rad_s=angular)
        snapshot = self.simulation.step(self.dt_s)
        self._draw(snapshot)
        self.line_label.configure(text=f"Line: {snapshot.line_binary}")
        self.obstacle_label.configure(text=f"IR: {list(snapshot.obstacle_binary)}")
        self._tick_job = self.window.after(int(self.dt_s * 1000), self._tick)

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        return self.simulation.board.world_to_canvas(x_m, y_m, self.canvas_size_px)

    def _meters_to_pixels(self, meters: float) -> float:
        return self.simulation.board.meters_to_pixels(meters, self.canvas_size_px)

    def _draw(self, snapshot: SensorSnapshot) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_obstacles()
        self._draw_robot(snapshot)

    def _draw_board(self) -> None:
        board = self.simulation.board
        cfg = board.config
        hx, hy = cfg.half_extent_x_m, cfg.half_extent_y_m
        x0, y0 = self._world_to_canvas(-hx, hy)
        x1, y1 = self._world_to_canvas(hx, -hy)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="#888", width=2)
        line_width = max(1, int(self._meters_to_pixels(cfg.line_width_m)))
        for center in board.line_centers_x():
            x0, y0 = self._world_to_canvas(center, cfg.line_extent_y_m)
            x1, y1 = self._world_to_canvas(center, -cfg.line_extent_y_m)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width)
        for center in board.line_centers_y():
            x0, y0 = self._world_to_canvas(-cfg.line_extent_x_m, center)
            x1, y1 = self._world_to_canvas(cfg.line_extent_x_m, center)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width)

    def _draw_obstacles(self) -> None:
        for obstacle in self.simulation.obstacles:
            x0, y0 = self._world_to_canvas(obstacle.min_x_m, obstacle.max_y_m)
            x1, y1 = self._world_to_canvas(obstacle.max_x_m, obstacle.min_y_m)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill="#c96c28", outline="#7a3d14", width=2)

    def _draw_robot(self, snapshot: SensorSnapshot) -> None:
        pose = self.simulation.robot.pose
        radius_px = self._meters_to_pixels(self.simulation.robot.config.radius_m)
        center_x, center_y = self._world_to_canvas(pose.x, pose.y)
        self.canvas.create_oval(
            center_x - radius_px,
            center_y - radius_px,
            center_x + radius_px,
            center_y + radius_px,
            fill="#2f8f46",
            outline="#1f4f28",
            width=3,
        )
        front_x = pose.x + self.simulation.robot.config.radius_m * math.cos(pose.yaw)
        front_y = pose.y + self.simulation.robot.config.radius_m * math.sin(pose.yaw)
        front_px = self._world_to_canvas(front_x, front_y)
        self.canvas.create_line(center_x, center_y, front_px[0], front_px[1], fill="white", width=4)
        for index, point in enumerate(snapshot.line_positions_m):
            px, py = self._world_to_canvas(point.x, point.y)
            color = "#1f1f1f" if snapshot.line_binary[index] else "#d9d9d9"
            self.canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=color, outline="#244")
        for index, point in enumerate(snapshot.obstacle_positions_m):
            px, py = self._world_to_canvas(point.x, point.y)
            detected = snapshot.obstacle_binary[index] == 1
            color = "#d01919" if detected else "#ffb000"
            self.canvas.create_oval(px - 7, py - 7, px + 7, py + 7, fill=color, outline="#330000")
            distance_m = snapshot.obstacle_distances_m[index]
            ray_len = distance_m if distance_m is not None else self.simulation.robot.config.obstacle_sensor_max_range_m
            end_x = point.x + ray_len * math.cos(pose.yaw)
            end_y = point.y + ray_len * math.sin(pose.yaw)
            ray_end_px = self._world_to_canvas(end_x, end_y)
            self.canvas.create_line(px, py, ray_end_px[0], ray_end_px[1], fill=color, dash=(4, 2), width=2)
