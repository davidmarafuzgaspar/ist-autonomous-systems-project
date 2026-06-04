"""Board, robot, obstacles, and simulation (no GUI)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

MIN_LINES = 3
MAX_LINES = 9
MIN_COLUMNS = 3
MAX_COLUMNS = 9
DEFAULT_LINES = 5
DEFAULT_COLUMNS = 5

BLACK = 1
WHITE = 0
OBSTACLE_SIZE_M = 0.08


@dataclass(frozen=True)
class BoardConfig:
    lines: int = DEFAULT_LINES
    columns: int = DEFAULT_COLUMNS
    spacing_m: float = 0.30
    line_width_m: float = 0.07
    outward_margin_m: float = 0.0
    outer_extension_m: float = 0.15
    margin_m: float = 0.12

    @staticmethod
    def line_extent_m(crosses: int, spacing_m: float, outer_extension_m: float) -> float:
        return (crosses - 1) * spacing_m / 2.0 + spacing_m / 2.0 + outer_extension_m

    @property
    def line_extent_x_m(self) -> float:
        return self.line_extent_m(self.columns, self.spacing_m, self.outer_extension_m)

    @property
    def line_extent_y_m(self) -> float:
        return self.line_extent_m(self.lines, self.spacing_m, self.outer_extension_m)

    @property
    def half_extent_x_m(self) -> float:
        return self.line_extent_x_m + self.outward_margin_m

    @property
    def half_extent_y_m(self) -> float:
        return self.line_extent_y_m + self.outward_margin_m

    @property
    def view_half_extent_m(self) -> float:
        return max(self.half_extent_x_m, self.half_extent_y_m) + self.margin_m


@dataclass(frozen=True)
class RobotConfig:
    radius_m: float = 0.055
    max_linear_speed_m_s: float = 0.25
    max_angular_speed_rad_s: float = 2.0
    line_sensor_count: int = 5
    line_sensor_spacing_m: float = 0.018
    line_sensor_local_x_m: float = 0.06
    line_black_value: int = 1000
    line_white_value: int = 0
    obstacle_sensor_local_x_m: float = 0.07
    obstacle_sensor_lateral_offset_m: float = 0.045
    obstacle_sensor_max_range_m: float = 0.35


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
        return [Point2D(x, y) for x in self.line_centers_x() for y in self.line_centers_y()]

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
        return (x_m + view) * scale, (view - y_m) * scale

    def meters_to_pixels(self, meters: float, canvas_px: int) -> float:
        view = self.config.view_half_extent_m
        return meters * canvas_px / (2.0 * view)

    def canvas_to_world(self, px: float, py: float, canvas_px: int) -> tuple[float, float]:
        view = self.config.view_half_extent_m
        scale = canvas_px / (2.0 * view)
        return px / scale - view, view - py / scale

    def default_start_pose(self) -> tuple[float, float, float]:
        xs = self.line_centers_x()
        ys = self.line_centers_y()
        return xs[0], ys[0], math.pi / 2.0


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
        return self.min_x_m <= point.x <= self.max_x_m and self.min_y_m <= point.y <= self.max_y_m


def obstacle_at_crossing(name: str, center: Point2D) -> RectangleObstacle:
    return RectangleObstacle(
        name=name,
        center_x_m=center.x,
        center_y_m=center.y,
        width_m=OBSTACLE_SIZE_M,
        height_m=OBSTACLE_SIZE_M,
    )


def snap_obstacles_to_board(
    obstacles: list[RectangleObstacle],
    board: CrossBoard,
) -> list[RectangleObstacle]:
    by_key: dict[tuple[float, float], RectangleObstacle] = {}
    for obstacle in obstacles:
        junction = board.nearest_crossing(obstacle.center_x_m, obstacle.center_y_m)
        if junction is None:
            continue
        by_key[board.crossing_key(junction)] = obstacle_at_crossing(obstacle.name, junction)
    return list(by_key.values())


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


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _transform_point(local_point: Point2D, pose: Pose2D) -> Point2D:
    cos_a = math.cos(pose.yaw)
    sin_a = math.sin(pose.yaw)
    return Point2D(
        x=pose.x + local_point.x * cos_a - local_point.y * sin_a,
        y=pose.y + local_point.x * sin_a + local_point.y * cos_a,
    )


@dataclass(frozen=True)
class RobotCommand:
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0


@dataclass(frozen=True)
class SensorSnapshot:
    line_binary: list[int]
    line_analog: list[int]
    line_positions_m: list[Point2D]
    obstacle_binary: tuple[int, int]
    obstacle_distances_m: tuple[float | None, float | None]
    obstacle_positions_m: tuple[Point2D, Point2D]


@dataclass
class AlphaBot2Robot:
    config: RobotConfig = field(default_factory=RobotConfig)
    pose: Pose2D = field(default_factory=lambda: Pose2D(x=-0.60, y=-0.60, yaw=0.0))
    command: RobotCommand = field(default_factory=RobotCommand)

    def reset(self, pose: Pose2D) -> None:
        self.pose = pose
        self.command = RobotCommand()

    def set_command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.command = RobotCommand(
            linear_m_s=_clamp(linear_m_s, -self.config.max_linear_speed_m_s, self.config.max_linear_speed_m_s),
            angular_rad_s=_clamp(
                angular_rad_s,
                -self.config.max_angular_speed_rad_s,
                self.config.max_angular_speed_rad_s,
            ),
        )

    def step(self, dt_s: float, board: CrossBoard, obstacles: list[RectangleObstacle]) -> None:
        next_pose = Pose2D(
            x=self.pose.x + self.command.linear_m_s * math.cos(self.pose.yaw) * dt_s,
            y=self.pose.y + self.command.linear_m_s * math.sin(self.pose.yaw) * dt_s,
            yaw=_wrap_angle(self.pose.yaw + self.command.angular_rad_s * dt_s),
        )
        center = Point2D(next_pose.x, next_pose.y)
        if not board.is_robot_inside(next_pose.x, next_pose.y, self.config.radius_m):
            self.command = RobotCommand()
            return
        if any(circle_intersects_rectangle(center, self.config.radius_m, o) for o in obstacles):
            self.command = RobotCommand()
            return
        self.pose = next_pose

    def line_sensor_positions(self) -> list[Point2D]:
        mid_index = (self.config.line_sensor_count - 1) / 2.0
        return [
            _transform_point(
                Point2D(
                    x=self.config.line_sensor_local_x_m,
                    y=(mid_index - index) * self.config.line_sensor_spacing_m,
                ),
                self.pose,
            )
            for index in range(self.config.line_sensor_count)
        ]

    def obstacle_sensor_positions(self) -> tuple[Point2D, Point2D]:
        return (
            _transform_point(
                Point2D(self.config.obstacle_sensor_local_x_m, self.config.obstacle_sensor_lateral_offset_m),
                self.pose,
            ),
            _transform_point(
                Point2D(self.config.obstacle_sensor_local_x_m, -self.config.obstacle_sensor_lateral_offset_m),
                self.pose,
            ),
        )

    def read_sensors(self, board: CrossBoard, obstacles: list[RectangleObstacle]) -> SensorSnapshot:
        line_positions = self.line_sensor_positions()
        line_binary = [1 if board.color_at(p.x, p.y) == BLACK else 0 for p in line_positions]
        line_analog = [
            self.config.line_black_value if v == 1 else self.config.line_white_value for v in line_binary
        ]
        left_pos, right_pos = self.obstacle_sensor_positions()
        direction = Point2D(x=math.cos(self.pose.yaw), y=math.sin(self.pose.yaw))
        left_distance = ray_distance_to_obstacles(
            left_pos, direction, self.config.obstacle_sensor_max_range_m, obstacles
        )
        right_distance = ray_distance_to_obstacles(
            right_pos, direction, self.config.obstacle_sensor_max_range_m, obstacles
        )
        return SensorSnapshot(
            line_binary=line_binary,
            line_analog=line_analog,
            line_positions_m=line_positions,
            obstacle_binary=(1 if left_distance is not None else 0, 1 if right_distance is not None else 0),
            obstacle_distances_m=(left_distance, right_distance),
            obstacle_positions_m=(left_pos, right_pos),
        )


@dataclass
class WorldSetup:
    lines: int
    columns: int
    obstacles: list[RectangleObstacle]


def default_world_setup() -> WorldSetup:
    return WorldSetup(lines=DEFAULT_LINES, columns=DEFAULT_COLUMNS, obstacles=[])


@dataclass
class AlphaBotSimulation:
    board: CrossBoard = field(default_factory=CrossBoard)
    robot: AlphaBot2Robot = field(default_factory=AlphaBot2Robot)
    obstacles: list[RectangleObstacle] = field(default_factory=list)
    time_s: float = 0.0

    @classmethod
    def from_setup(cls, setup: WorldSetup) -> AlphaBotSimulation:
        board = CrossBoard(config=BoardConfig(lines=setup.lines, columns=setup.columns))
        sx, sy, yaw = board.default_start_pose()
        return cls(
            board=board,
            robot=AlphaBot2Robot(config=RobotConfig(), pose=Pose2D(x=sx, y=sy, yaw=yaw)),
            obstacles=snap_obstacles_to_board(list(setup.obstacles), board),
        )

    def to_setup(self) -> WorldSetup:
        return WorldSetup(
            lines=self.board.config.lines,
            columns=self.board.config.columns,
            obstacles=list(self.obstacles),
        )

    def reset(self) -> None:
        sx, sy, yaw = self.board.default_start_pose()
        self.robot.reset(Pose2D(x=sx, y=sy, yaw=yaw))
        self.time_s = 0.0

    def step(self, dt_s: float) -> SensorSnapshot:
        self.robot.step(dt_s=dt_s, board=self.board, obstacles=self.obstacles)
        self.time_s += dt_s
        return self.robot.read_sensors(board=self.board, obstacles=self.obstacles)

    def set_command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.robot.set_command(linear_m_s=linear_m_s, angular_rad_s=angular_rad_s)
