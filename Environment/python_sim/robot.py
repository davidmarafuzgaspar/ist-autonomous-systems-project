from __future__ import annotations

from dataclasses import dataclass, field
import math

from .board import BLACK, CrossBoard, WhiteCell
from .config import RobotConfig
from .geometry import Point2D, Pose2D, clamp, transform_point, wrap_angle
from .obstacles import RectangleObstacle, circle_intersects_rectangle, ray_distance_to_obstacles


@dataclass(frozen=True)
class RobotCommand:
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0


@dataclass(frozen=True)
class CameraDetection:
    marker_id: int
    cell_row: int
    cell_col: int
    marker_position_m: Point2D
    distance_m: float
    bearing_rad: float


@dataclass(frozen=True)
class CellLocalization:
    marker_id: int
    cell_row: int
    cell_col: int


@dataclass(frozen=True)
class SensorSnapshot:
    line_binary: list[int]
    line_analog: list[int]
    line_positions_m: list[Point2D]
    obstacle_binary: tuple[int, int]
    obstacle_distances_m: tuple[float | None, float | None]
    obstacle_positions_m: tuple[Point2D, Point2D]
    camera_position_m: Point2D
    camera_yaw_rad: float
    camera_visible_markers: list[CameraDetection]
    localized_cell: CellLocalization | None


@dataclass
class AlphaBot2Robot:
    config: RobotConfig = field(default_factory=RobotConfig)
    pose: Pose2D = field(default_factory=lambda: Pose2D(x=-0.50, y=-0.075, yaw=0.0))
    command: RobotCommand = field(default_factory=RobotCommand)

    def reset(self, pose: Pose2D) -> None:
        self.pose = pose
        self.command = RobotCommand()

    def set_command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.command = RobotCommand(
            linear_m_s=clamp(
                linear_m_s,
                -self.config.max_linear_speed_m_s,
                self.config.max_linear_speed_m_s,
            ),
            angular_rad_s=clamp(
                angular_rad_s,
                -self.config.max_angular_speed_rad_s,
                self.config.max_angular_speed_rad_s,
            ),
        )

    def step(self, dt_s: float, board: CrossBoard, obstacles: list[RectangleObstacle]) -> None:
        next_pose = Pose2D(
            x=self.pose.x + self.command.linear_m_s * math.cos(self.pose.yaw) * dt_s,
            y=self.pose.y + self.command.linear_m_s * math.sin(self.pose.yaw) * dt_s,
            yaw=wrap_angle(self.pose.yaw + self.command.angular_rad_s * dt_s),
        )

        center = Point2D(next_pose.x, next_pose.y)
        if not board.is_robot_inside(next_pose.x, next_pose.y, self.config.radius_m):
            self.command = RobotCommand()
            return

        if any(circle_intersects_rectangle(center, self.config.radius_m, obstacle) for obstacle in obstacles):
            self.command = RobotCommand()
            return

        self.pose = next_pose

    def line_sensor_positions(self) -> list[Point2D]:
        mid_index = (self.config.line_sensor_count - 1) / 2.0
        positions = []
        for index in range(self.config.line_sensor_count):
            lateral_offset = (mid_index - index) * self.config.line_sensor_spacing_m
            local = Point2D(
                x=self.config.line_sensor_local_x_m,
                y=lateral_offset,
            )
            positions.append(transform_point(local, self.pose))
        return positions

    def obstacle_sensor_positions(self) -> tuple[Point2D, Point2D]:
        local_left = Point2D(
            x=self.config.obstacle_sensor_local_x_m,
            y=self.config.obstacle_sensor_lateral_offset_m,
        )
        local_right = Point2D(
            x=self.config.obstacle_sensor_local_x_m,
            y=-self.config.obstacle_sensor_lateral_offset_m,
        )
        return transform_point(local_left, self.pose), transform_point(local_right, self.pose)

    def camera_pose(self) -> tuple[Point2D, float]:
        local_camera = Point2D(
            x=self.config.camera_local_x_m,
            y=self.config.camera_local_y_m,
        )
        return (
            transform_point(local_camera, self.pose),
            wrap_angle(self.pose.yaw + self.config.camera_yaw_offset_rad),
        )

    def visible_markers(
        self,
        board: CrossBoard,
        camera_position_m: Point2D,
        camera_yaw_rad: float,
    ) -> list[CameraDetection]:
        visible_markers: list[CameraDetection] = []

        for marker in board.markers():
            detection = self._marker_detection(marker, camera_position_m, camera_yaw_rad)
            if detection is not None:
                visible_markers.append(detection)

        visible_markers.sort(key=lambda item: (item.distance_m, abs(item.bearing_rad), item.marker_id))
        return visible_markers

    def _marker_detection(
        self,
        marker: WhiteCell,
        camera_position_m: Point2D,
        camera_yaw_rad: float,
    ) -> CameraDetection | None:
        dx = marker.center_m.x - camera_position_m.x
        dy = marker.center_m.y - camera_position_m.y
        distance_m = math.hypot(dx, dy)
        if distance_m > self.config.camera_max_range_m:
            return None

        bearing_rad = wrap_angle(math.atan2(dy, dx) - camera_yaw_rad)
        if abs(bearing_rad) > self.config.camera_fov_rad / 2.0:
            return None

        return CameraDetection(
            marker_id=marker.marker_id,
            cell_row=marker.row,
            cell_col=marker.col,
            marker_position_m=marker.center_m,
            distance_m=distance_m,
            bearing_rad=bearing_rad,
        )

    def read_sensors(self, board: CrossBoard, obstacles: list[RectangleObstacle]) -> SensorSnapshot:
        line_positions = self.line_sensor_positions()
        line_binary = [1 if board.color_at(pos.x, pos.y) == BLACK else 0 for pos in line_positions]
        line_analog = [
            self.config.line_black_value if value == 1 else self.config.line_white_value
            for value in line_binary
        ]

        left_pos, right_pos = self.obstacle_sensor_positions()
        direction = Point2D(x=math.cos(self.pose.yaw), y=math.sin(self.pose.yaw))

        left_distance = ray_distance_to_obstacles(
            origin=left_pos,
            direction=direction,
            max_distance_m=self.config.obstacle_sensor_max_range_m,
            obstacles=obstacles,
        )
        right_distance = ray_distance_to_obstacles(
            origin=right_pos,
            direction=direction,
            max_distance_m=self.config.obstacle_sensor_max_range_m,
            obstacles=obstacles,
        )

        camera_position_m, camera_yaw_rad = self.camera_pose()
        camera_visible_markers = self.visible_markers(
            board=board,
            camera_position_m=camera_position_m,
            camera_yaw_rad=camera_yaw_rad,
        )
        localized_cell = None
        if camera_visible_markers:
            best_marker = camera_visible_markers[0]
            localized_cell = CellLocalization(
                marker_id=best_marker.marker_id,
                cell_row=best_marker.cell_row,
                cell_col=best_marker.cell_col,
            )

        return SensorSnapshot(
            line_binary=line_binary,
            line_analog=line_analog,
            line_positions_m=line_positions,
            obstacle_binary=(
                1 if left_distance is not None else 0,
                1 if right_distance is not None else 0,
            ),
            obstacle_distances_m=(left_distance, right_distance),
            obstacle_positions_m=(left_pos, right_pos),
            camera_position_m=camera_position_m,
            camera_yaw_rad=camera_yaw_rad,
            camera_visible_markers=camera_visible_markers,
            localized_cell=localized_cell,
        )

