from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import BoardConfig


BLACK = 1
WHITE = 0


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


def line_centers_for_axis(crosses: int, spacing_m: float) -> list[float]:
    half_span = (crosses - 1) / 2.0
    return [(index - half_span) * spacing_m for index in range(crosses)]


@dataclass
class CrossBoard:
    config: BoardConfig = field(default_factory=BoardConfig)

    def line_centers_x(self) -> list[float]:
        return line_centers_for_axis(self.config.columns, self.config.spacing_m)

    def line_centers_y(self) -> list[float]:
        return line_centers_for_axis(self.config.lines, self.config.spacing_m)

    def is_inside(self, x_m: float, y_m: float) -> bool:
        return (
            -self.config.half_extent_x_m <= x_m <= self.config.half_extent_x_m
            and -self.config.half_extent_y_m <= y_m <= self.config.half_extent_y_m
        )

    def is_robot_inside(self, x_m: float, y_m: float, radius_m: float) -> bool:
        hx = self.config.half_extent_x_m - radius_m
        hy = self.config.half_extent_y_m - radius_m
        return -hx <= x_m <= hx and -hy <= y_m <= hy

    def is_line_at(self, x_m: float, y_m: float) -> bool:
        if not self.is_inside(x_m, y_m):
            return False
        half_width = self.config.line_width_m / 2.0
        tolerance = 1e-6
        if (
            abs(x_m) > self.config.line_extent_x_m + tolerance
            or abs(y_m) > self.config.line_extent_y_m + tolerance
        ):
            return False
        on_vertical = any(
            abs(x_m - center) <= half_width + tolerance for center in self.line_centers_x()
        )
        on_horizontal = any(
            abs(y_m - center) <= half_width + tolerance for center in self.line_centers_y()
        )
        return on_vertical or on_horizontal

    def color_at(self, x_m: float, y_m: float) -> int:
        return BLACK if self.is_line_at(x_m, y_m) else WHITE

    def crossing_points(self) -> list[Point2D]:
        return [
            Point2D(x, y)
            for x in self.line_centers_x()
            for y in self.line_centers_y()
        ]

    def nearest_crossing(self, x_m: float, y_m: float, *, max_dist_m: float | None = None) -> Point2D | None:
        if max_dist_m is None:
            max_dist_m = self.config.spacing_m * 0.45
        best: Point2D | None = None
        best_dist = max_dist_m
        for point in self.crossing_points():
            dist = math.hypot(point.x - x_m, point.y - y_m)
            if dist < best_dist:
                best_dist = dist
                best = point
        return best

    def crossing_key(self, point: Point2D) -> tuple[float, float]:
        return (round(point.x, 4), round(point.y, 4))

    def world_to_canvas(self, x_m: float, y_m: float, canvas_px: int) -> tuple[float, float]:
        view = self.config.view_half_extent_m
        scale = canvas_px / (2.0 * view)
        x_px = (x_m + view) * scale
        y_px = (view - y_m) * scale
        return x_px, y_px

    def meters_to_pixels(self, meters: float, canvas_px: int) -> float:
        view = self.config.view_half_extent_m
        return meters * canvas_px / (2.0 * view)

    def canvas_to_world(self, px: float, py: float, canvas_px: int) -> tuple[float, float]:
        view = self.config.view_half_extent_m
        scale = canvas_px / (2.0 * view)
        x_m = px / scale - view
        y_m = view - py / scale
        return x_m, y_m

    def default_start_pose(self) -> tuple[float, float, float]:
        xs = self.line_centers_x()
        ys = self.line_centers_y()
        return xs[0], ys[0], math.pi / 2.0
