from __future__ import annotations

import math
import tkinter as tk

from .robot import SensorSnapshot
from .simulation import AlphaBotSimulation


class SimulationViewer:
    def __init__(self, simulation: AlphaBotSimulation, dt_s: float = 0.05) -> None:
        self.simulation = simulation
        self.dt_s = dt_s
        self.window = tk.Tk()
        self.window.title("AlphaBot2 Pure Python Simulator")
        self.canvas_size_px = 900
        self.sidebar_width_px = 270
        self.canvas = tk.Canvas(
            self.window,
            width=self.canvas_size_px + self.sidebar_width_px,
            height=self.canvas_size_px,
            bg="#f0f0f0",
            highlightthickness=0,
        )
        self.canvas.pack()

        self.keys_pressed: set[str] = set()
        self.window.bind("<KeyPress>", self._on_key_press)
        self.window.bind("<KeyRelease>", self._on_key_release)

    def run(self) -> None:
        self._tick()
        self.window.mainloop()

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
        self.window.after(int(self.dt_s * 1000), self._tick)

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        extent = self.simulation.board.config.half_extent_m + self.simulation.board.config.margin_m
        scale = self.canvas_size_px / (2.0 * extent)
        x_px = (x_m + extent) * scale
        y_px = (extent - y_m) * scale
        return x_px, y_px

    def _meters_to_pixels(self, meters: float) -> float:
        extent = self.simulation.board.config.half_extent_m + self.simulation.board.config.margin_m
        return meters * self.canvas_size_px / (2.0 * extent)

    def _draw(self, snapshot: SensorSnapshot) -> None:
        self.canvas.delete("all")
        self._draw_board()
        self._draw_markers(snapshot)
        self._draw_obstacles()
        self._draw_robot(snapshot)
        self._draw_sidebar(snapshot)

    def _draw_board(self) -> None:
        half = self.simulation.board.config.half_extent_m
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="#888", width=2)

        line_width = self._meters_to_pixels(self.simulation.board.config.line_width_m)
        line_extent = self.simulation.board.config.line_extent_m
        for center in self.simulation.board.line_centers():
            x0, y0 = self._world_to_canvas(center, line_extent)
            x1, y1 = self._world_to_canvas(center, -line_extent)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width)

            x0, y0 = self._world_to_canvas(-line_extent, center)
            x1, y1 = self._world_to_canvas(line_extent, center)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width)

    def _draw_markers(self, snapshot: SensorSnapshot) -> None:
        visible_marker_ids = {marker.marker_id for marker in snapshot.camera_visible_markers}
        localized_marker_id = None
        if snapshot.localized_cell is not None:
            localized_marker_id = snapshot.localized_cell.marker_id

        marker_half_size_px = max(8.0, self._meters_to_pixels(self.simulation.board.white_cell_size_m() * 0.16))
        for marker in self.simulation.white_cells():
            px, py = self._world_to_canvas(marker.center_m.x, marker.center_m.y)

            fill = "#f4f4f4"
            outline = "#8a8a8a"
            if marker.marker_id in visible_marker_ids:
                fill = "#d6f5d6"
                outline = "#2a8a2a"
            if marker.marker_id == localized_marker_id:
                fill = "#bfe7ff"
                outline = "#1565c0"

            self.canvas.create_rectangle(
                px - marker_half_size_px,
                py - marker_half_size_px,
                px + marker_half_size_px,
                py + marker_half_size_px,
                fill=fill,
                outline=outline,
                width=2,
            )
            self.canvas.create_text(
                px,
                py,
                text=str(marker.marker_id),
                fill="#333",
                font=("Courier New", 8, "bold"),
            )

    def _draw_obstacles(self) -> None:
        for obstacle in self.simulation.obstacles:
            x0, y0 = self._world_to_canvas(obstacle.min_x_m, obstacle.max_y_m)
            x1, y1 = self._world_to_canvas(obstacle.max_x_m, obstacle.min_y_m)
            self.canvas.create_rectangle(x0, y0, x1, y1, fill="#c96c28", outline="#7a3d14", width=2)

    def _draw_robot(self, snapshot: SensorSnapshot) -> None:
        pose = self.simulation.robot.pose
        radius_px = self._meters_to_pixels(self.simulation.robot.config.radius_m)
        center_x, center_y = self._world_to_canvas(pose.x, pose.y)

        self._draw_camera(snapshot)

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

    def _draw_camera(self, snapshot: SensorSnapshot) -> None:
        camera_position = snapshot.camera_position_m
        camera_px = self._world_to_canvas(camera_position.x, camera_position.y)
        fov = self.simulation.robot.config.camera_fov_rad
        max_range_m = self.simulation.robot.config.camera_max_range_m

        wedge_points = [camera_px]
        ray_samples = 12
        for index in range(ray_samples + 1):
            angle = snapshot.camera_yaw_rad - fov / 2.0 + index * fov / ray_samples
            end_x = camera_position.x + max_range_m * math.cos(angle)
            end_y = camera_position.y + max_range_m * math.sin(angle)
            wedge_points.append(self._world_to_canvas(end_x, end_y))

        flattened_points = [value for point in wedge_points for value in point]
        self.canvas.create_polygon(
            flattened_points,
            fill="",
            outline="#2979ff",
            width=2,
        )
        self.canvas.create_oval(
            camera_px[0] - 5,
            camera_px[1] - 5,
            camera_px[0] + 5,
            camera_px[1] + 5,
            fill="#2979ff",
            outline="#0d47a1",
        )

    def _draw_sidebar(self, snapshot: SensorSnapshot) -> None:
        left = self.canvas_size_px + 20
        self.canvas.create_text(
            left,
            40,
            anchor="w",
            text="AlphaBot2 Sensors",
            font=("Arial", 16, "bold"),
            fill="#222",
        )

        info_lines = [
            "line sensors:",
            f"binary: {snapshot.line_binary}",
            "",
            "front obstacle sensors:",
            f"binary: {list(snapshot.obstacle_binary)}",
            "",
            "camera localization:",
            self._localized_cell_text(snapshot),
        ]

        y = 90
        for line in info_lines:
            self.canvas.create_text(
                left,
                y,
                anchor="w",
                text=line,
                font=("Courier New", 11),
                fill="#333",
            )
            y += 24

    def _localized_cell_text(self, snapshot: SensorSnapshot) -> str:
        if snapshot.localized_cell is None:
            return "localized: --"
        return f"via marker {snapshot.localized_cell.marker_id}"
