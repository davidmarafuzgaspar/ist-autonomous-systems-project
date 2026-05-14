from __future__ import annotations

import math

from dataclasses import dataclass, field

from .board import CrossBoard, WhiteCell
from .config import BoardConfig, RobotConfig
from .geometry import Pose2D
from .obstacles import RectangleObstacle
from .robot import AlphaBot2Robot, SensorSnapshot


def default_obstacles() -> list[RectangleObstacle]:
    obstacle_size_m = 0.08
    intersections = [
        ("cross_left_mid", -0.60, 0.0),
        ("cross_top_left_inner", -0.30, 0.60),
        ("cross_mid_right", 0.30, 0.0),
        ("cross_bottom_mid", 0.0, -0.30),
        ("cross_right_lower", 0.60, -0.30),
    ]
    return [
        RectangleObstacle(
            name=name,
            center_x_m=center_x,
            center_y_m=center_y,
            width_m=obstacle_size_m,
            height_m=obstacle_size_m,
        )
        for name, center_x, center_y in intersections
    ]


@dataclass
class AlphaBotSimulation:
    board: CrossBoard = field(default_factory=CrossBoard)
    robot: AlphaBot2Robot = field(default_factory=AlphaBot2Robot)
    obstacles: list[RectangleObstacle] = field(default_factory=default_obstacles)
    time_s: float = 0.0

    @classmethod
    def create_default(cls) -> "AlphaBotSimulation":
        board = CrossBoard(config=BoardConfig())
        robot = AlphaBot2Robot(
            config=RobotConfig(),
            pose=Pose2D(x=-0.60, y=-0.60, yaw=math.pi / 2.0),
        )
        return cls(board=board, robot=robot, obstacles=default_obstacles())

    def reset(self) -> None:
        self.robot.reset(Pose2D(x=-0.60, y=-0.60, yaw=math.pi / 2.0))
        self.time_s = 0.0

    def step(self, dt_s: float) -> SensorSnapshot:
        self.robot.step(dt_s=dt_s, board=self.board, obstacles=self.obstacles)
        self.time_s += dt_s
        return self.read_sensors()

    def read_sensors(self) -> SensorSnapshot:
        return self.robot.read_sensors(board=self.board, obstacles=self.obstacles)

    def set_command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.robot.set_command(linear_m_s=linear_m_s, angular_rad_s=angular_rad_s)

    def markers(self) -> list[WhiteCell]:
        return self.board.markers()

