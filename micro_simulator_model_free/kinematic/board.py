from __future__ import annotations

from dataclasses import dataclass, field

from .config import BoardConfig


BLACK = 1
WHITE = 0


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class WhiteCell:
    row: int
    col: int
    marker_id: int
    center_m: Point2D


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

    def white_cell_size_m(self) -> float:
        return self.config.spacing_m - self.config.line_width_m

    def white_cell_centers(self) -> tuple[list[float], list[float]]:
        centers = self.line_centers()
        line_extent = self.config.line_extent_m

        x_centers: list[float] = [(-line_extent + centers[0]) / 2.0]
        for index in range(len(centers) - 1):
            x_centers.append((centers[index] + centers[index + 1]) / 2.0)
        x_centers.append((centers[-1] + line_extent) / 2.0)

        y_centers: list[float] = [(centers[-1] + line_extent) / 2.0]
        for index in range(len(centers) - 2, -1, -1):
            y_centers.append((centers[index] + centers[index + 1]) / 2.0)
        y_centers.append((-line_extent + centers[0]) / 2.0)

        return x_centers, y_centers

    def white_cells(self) -> list[WhiteCell]:
        x_centers, y_centers = self.white_cell_centers()
        cells: list[WhiteCell] = []
        marker_id = 0
        for row, center_y in enumerate(y_centers):
            for col, center_x in enumerate(x_centers):
                cells.append(
                    WhiteCell(
                        row=row,
                        col=col,
                        marker_id=marker_id,
                        center_m=Point2D(center_x, center_y),
                    )
                )
                marker_id += 1
        return cells

    def is_line_at(self, x_m: float, y_m: float) -> bool:
        if not self.is_inside(x_m, y_m):
            return False
        half_width = self.config.line_width_m / 2.0
        tolerance = 1e-6
        line_extent = self.config.line_extent_m
        if abs(x_m) > line_extent + tolerance or abs(y_m) > line_extent + tolerance:
            return False
        on_vertical = any(abs(x_m - center) <= half_width + tolerance for center in self.line_centers())
        on_horizontal = any(abs(y_m - center) <= half_width + tolerance for center in self.line_centers())
        return on_vertical or on_horizontal

    def color_at(self, x_m: float, y_m: float) -> int:
        return BLACK if self.is_line_at(x_m, y_m) else WHITE
