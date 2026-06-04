from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .board import CrossBoard, Point2D

OBSTACLE_SIZE_M = 0.08


def obstacle_at_crossing(name: str, center: Point2D) -> "RectangleObstacle":
    return RectangleObstacle(
        name=name,
        center_x_m=center.x,
        center_y_m=center.y,
        width_m=OBSTACLE_SIZE_M,
        height_m=OBSTACLE_SIZE_M,
    )


def snap_obstacles_to_board(
    obstacles: list["RectangleObstacle"],
    board: CrossBoard,
) -> list["RectangleObstacle"]:
    by_key: dict[tuple[float, float], RectangleObstacle] = {}
    for obstacle in obstacles:
        junction = board.nearest_crossing(obstacle.center_x_m, obstacle.center_y_m)
        if junction is None:
            continue
        key = board.crossing_key(junction)
        by_key[key] = obstacle_at_crossing(obstacle.name, junction)
    return list(by_key.values())


@dataclass(frozen=True)
class RectangleObstacle:
    name: str
    center_x_m: float
    center_y_m: float
    width_m: float
    height_m: float

    @property
    def min_x_m(self) -> float:
        return self.center_x_m - self.width_m / 2.0

    @property
    def max_x_m(self) -> float:
        return self.center_x_m + self.width_m / 2.0

    @property
    def min_y_m(self) -> float:
        return self.center_y_m - self.height_m / 2.0

    @property
    def max_y_m(self) -> float:
        return self.center_y_m + self.height_m / 2.0

    def contains_point(self, point: Point2D) -> bool:
        return (
            self.min_x_m <= point.x <= self.max_x_m
            and self.min_y_m <= point.y <= self.max_y_m
        )


def ray_distance_to_obstacles(
    origin: Point2D,
    direction: Point2D,
    max_distance_m: float,
    obstacles: Iterable[RectangleObstacle],
    step_m: float = 0.002,
) -> float | None:
    distance_m = 0.0
    while distance_m <= max_distance_m:
        sample = Point2D(
            x=origin.x + direction.x * distance_m,
            y=origin.y + direction.y * distance_m,
        )
        for obstacle in obstacles:
            if obstacle.contains_point(sample):
                return distance_m
        distance_m += step_m
    return None


def circle_intersects_rectangle(
    center: Point2D,
    radius_m: float,
    obstacle: RectangleObstacle,
) -> bool:
    closest_x = min(max(center.x, obstacle.min_x_m), obstacle.max_x_m)
    closest_y = min(max(center.y, obstacle.min_y_m), obstacle.max_y_m)
    dx = center.x - closest_x
    dy = center.y - closest_y
    return dx * dx + dy * dy <= radius_m * radius_m
