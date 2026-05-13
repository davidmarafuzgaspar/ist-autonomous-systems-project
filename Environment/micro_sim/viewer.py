"""Tkinter viewer that renders the cross board with policy arrows.

Draws the same cross-shaped board used by ``python_sim`` and overlays
an arrow on every intersection showing the optimal action from that
state. Obstacles, start and goal are highlighted.
"""

from __future__ import annotations

import tkinter as tk

from .world import Action, GridCell, IntersectionWorld


LINE_WIDTH_M = 0.07
OUTER_EXTENSION_M = 0.15
MARGIN_M = 0.12
OBSTACLE_SIZE_M = 0.08

ARROW_COLOR = "#ffeb3b"
ARROW_OUTLINE = "#5d4e00"
START_COLOR = "#2f8f46"
START_OUTLINE = "#1f4f28"
GOAL_COLOR = "#1565c0"
GOAL_OUTLINE = "#0d47a1"
OBSTACLE_FILL = "#c96c28"
OBSTACLE_OUTLINE = "#7a3d14"


_ACTION_UNIT_VEC: dict[Action, tuple[float, float]] = {
    Action.UP: (0.0, 1.0),
    Action.DOWN: (0.0, -1.0),
    Action.LEFT: (-1.0, 0.0),
    Action.RIGHT: (1.0, 0.0),
}


class PolicyViewer:
    def __init__(
        self,
        world: IntersectionWorld,
        policy: dict[GridCell, Action | None],
        values: dict[GridCell, float] | None = None,
        canvas_size_px: int = 800,
        title: str = "Micro Sim - Optimal Policy",
        show_values: bool = False,
    ) -> None:
        self.world = world
        self.policy = policy
        self.values = values or {}
        self.canvas_size_px = canvas_size_px
        self.show_values = show_values

        self.window = tk.Tk()
        self.window.title(title)
        self.canvas = tk.Canvas(
            self.window,
            width=canvas_size_px,
            height=canvas_size_px,
            bg="#f0f0f0",
            highlightthickness=0,
        )
        self.canvas.pack()

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

    def run(self) -> None:
        self._draw()
        self.window.mainloop()

    def _draw(self) -> None:
        self._draw_board()
        self._draw_obstacles()
        self._draw_start_and_goal()
        self._draw_policy_arrows()

    def _draw_board(self) -> None:
        half = self._half_extent_m
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="#888", width=2)

        line_width_px = self._meters_to_pixels(LINE_WIDTH_M)
        line_extent = half
        for center in self._line_centers():
            x0, y0 = self._world_to_canvas(center, line_extent)
            x1, y1 = self._world_to_canvas(center, -line_extent)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)

            x0, y0 = self._world_to_canvas(-line_extent, center)
            x1, y1 = self._world_to_canvas(line_extent, center)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=line_width_px)

    def _line_centers(self) -> list[float]:
        n = self.world.cols
        half_span = (n - 1) / 2.0
        return [(index - half_span) * self.world.spacing_m for index in range(n)]

    def _draw_obstacles(self) -> None:
        half = OBSTACLE_SIZE_M / 2.0
        for cell in self.world.obstacles:
            x_m, y_m = self.world.world_xy(cell)
            x0, y0 = self._world_to_canvas(x_m - half, y_m + half)
            x1, y1 = self._world_to_canvas(x_m + half, y_m - half)
            self.canvas.create_rectangle(
                x0, y0, x1, y1,
                fill=OBSTACLE_FILL,
                outline=OBSTACLE_OUTLINE,
                width=2,
            )

    def _draw_policy_arrows(self) -> None:
        arrow_length_m = self.world.spacing_m * 0.4
        arrow_width_px = max(3.0, self._meters_to_pixels(0.012))
        head_long = max(10.0, self._meters_to_pixels(0.025))
        head_short = max(12.0, self._meters_to_pixels(0.03))
        head_wide = max(6.0, self._meters_to_pixels(0.018))

        for cell, action in self.policy.items():
            if action is None:
                continue
            if cell == self.world.goal:
                continue
            x_m, y_m = self.world.world_xy(cell)
            unit_x, unit_y = _ACTION_UNIT_VEC[action]
            start_px = self._world_to_canvas(
                x_m - unit_x * arrow_length_m / 2.0,
                y_m - unit_y * arrow_length_m / 2.0,
            )
            end_px = self._world_to_canvas(
                x_m + unit_x * arrow_length_m / 2.0,
                y_m + unit_y * arrow_length_m / 2.0,
            )
            self.canvas.create_line(
                start_px[0], start_px[1], end_px[0], end_px[1],
                fill=ARROW_COLOR,
                width=arrow_width_px,
                arrow=tk.LAST,
                arrowshape=(head_long, head_short, head_wide),
                capstyle=tk.ROUND,
            )

            if self.show_values and cell in self.values:
                label_offset_px = self._meters_to_pixels(self.world.spacing_m * 0.18)
                center_px = self._world_to_canvas(x_m, y_m)
                self.canvas.create_text(
                    center_px[0] + label_offset_px,
                    center_px[1] - label_offset_px,
                    text=f"{self.values[cell]:.0f}",
                    fill=ARROW_COLOR,
                    font=("Courier New", 9, "bold"),
                )

    def _draw_start_and_goal(self) -> None:
        radius_px = max(14.0, self._meters_to_pixels(0.07))
        label_offset_px = radius_px + 14

        start_x_m, start_y_m = self.world.world_xy(self.world.start)
        sx, sy = self._world_to_canvas(start_x_m, start_y_m)
        self.canvas.create_oval(
            sx - radius_px, sy - radius_px,
            sx + radius_px, sy + radius_px,
            fill="",
            outline=START_COLOR,
            width=4,
        )
        self.canvas.create_text(
            sx, sy + label_offset_px,
            text="S",
            fill=START_COLOR,
            font=("Arial", 12, "bold"),
        )

        goal_x_m, goal_y_m = self.world.world_xy(self.world.goal)
        gx, gy = self._world_to_canvas(goal_x_m, goal_y_m)
        self.canvas.create_oval(
            gx - radius_px, gy - radius_px,
            gx + radius_px, gy + radius_px,
            fill=GOAL_COLOR,
            outline=GOAL_OUTLINE,
            width=2,
        )
        self.canvas.create_text(gx, gy, text="G", fill="white", font=("Arial", 14, "bold"))
