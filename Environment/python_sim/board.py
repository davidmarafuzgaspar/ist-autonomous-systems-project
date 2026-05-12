from __future__ import annotations

from dataclasses import dataclass, field

from .config import BoardConfig


BLACK = 1
WHITE = 0


@dataclass
class CrossBoard:
    config: BoardConfig = field(default_factory=BoardConfig)

    def line_centers(self) -> list[float]:
        half_span = (self.config.crosses_per_axis - 1) / 2.0
        return [
            (index - half_span) * self.config.spacing_m
            for index in range(self.config.crosses_per_axis)
        ]

    def is_inside(self, x_m: float, y_m: float) -> bool:
        half = self.config.half_extent_m
        return -half <= x_m <= half and -half <= y_m <= half

    def is_robot_inside(self, x_m: float, y_m: float, radius_m: float) -> bool:
        half = self.config.half_extent_m - radius_m
        return -half <= x_m <= half and -half <= y_m <= half

    def is_line_at(self, x_m: float, y_m: float) -> bool:
        if not self.is_inside(x_m, y_m):
            return False

        half_width = self.config.line_width_m / 2.0
        tolerance = 1e-6
        on_vertical = any(abs(x_m - center) <= half_width + tolerance for center in self.line_centers())
        on_horizontal = any(abs(y_m - center) <= half_width + tolerance for center in self.line_centers())
        return on_vertical or on_horizontal

    def color_at(self, x_m: float, y_m: float) -> int:
        return BLACK if self.is_line_at(x_m, y_m) else WHITE

