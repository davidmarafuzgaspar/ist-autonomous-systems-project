from __future__ import annotations

import math
import tkinter as tk

from . import ui_theme as ui

from .robot import SensorSnapshot
from .simulation import AlphaBotSimulation


class SimulationViewer:
    def __init__(self, simulation: AlphaBotSimulation, dt_s: float = 0.05) -> None:
        self.simulation = simulation
        self.dt_s = dt_s
        self.change_world_requested = False
        self._tick_job: str | None = None

        self.window = tk.Tk()
        self.window.title("AlphaBot2 Simulator")
        ui.setup(self.window)

        body = tk.Frame(self.window, bg=ui.BG)
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

        self.sidebar = tk.Frame(body, bg=ui.BG, width=260)
        self.sidebar.pack(side=tk.RIGHT, fill="y", padx=12, pady=12)
        self.sidebar.pack_propagate(False)
        self._build_sidebar()

        self.keys_pressed: set[str] = set()
        self.window.bind("<KeyPress>", self._on_key_press)
        self.window.bind("<KeyRelease>", self._on_key_release)

    def _build_sidebar(self) -> None:
        title = ui.label(self.sidebar, "AlphaBot2")
        title.configure(font=ui.FONT_TITLE)
        title.pack(anchor="w", pady=(0, 8))

        cfg = self.simulation.board.config
        ui.label(
            self.sidebar,
            f"Grid: {cfg.lines}×{cfg.columns}",
            muted=True,
        ).pack(anchor="w", pady=(0, 4))
        ui.label(
            self.sidebar,
            f"Obstacles: {len(self.simulation.obstacles)}",
            muted=True,
        ).pack(anchor="w", pady=(0, 12))

        ui.label(self.sidebar, "Controls", muted=True).pack(anchor="w")
        ui.label(self.sidebar, "WASD / arrows - drive", muted=True).pack(anchor="w")
        ui.label(self.sidebar, "R - reset pose", muted=True).pack(anchor="w", pady=(0, 16))

        self.line_label = ui.label(self.sidebar, "Line: —")
        self.line_label.pack(anchor="w")
        self.obstacle_label = ui.label(self.sidebar, "IR: —")
        self.obstacle_label.pack(anchor="w", pady=(0, 16))

        ui.line(self.sidebar)
        ui.button(self.sidebar, "Change world", self._on_change_world).pack(fill="x", pady=(12, 0))

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
        linear = 0.0
        angular = 0.0

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
