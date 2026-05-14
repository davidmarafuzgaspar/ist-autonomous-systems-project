from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_point(point: Point2D, angle: float) -> Point2D:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return Point2D(
        x=point.x * cos_a - point.y * sin_a,
        y=point.x * sin_a + point.y * cos_a,
    )


def transform_point(local_point: Point2D, pose: Pose2D) -> Point2D:
    rotated = rotate_point(local_point, pose.yaw)
    return Point2D(x=pose.x + rotated.x, y=pose.y + rotated.y)


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

