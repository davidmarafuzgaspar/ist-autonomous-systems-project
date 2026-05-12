from __future__ import annotations

from dataclasses import dataclass, field

from .board import CrossBoard
from .config import BoardConfig, RobotConfig
from .geometry import Pose2D
from .obstacles import RectangleObstacle
from .robot import AlphaBot2Robot, SensorSnapshot


def default_obstacles() -> list[RectangleObstacle]:
    return [
        RectangleObstacle(
            name="front_box",
            center_x_m=0.12,
            center_y_m=-0.075,
            width_m=0.06,
            height_m=0.06,
        ),
        RectangleObstacle(
            name="top_box",
            center_x_m=-0.075,
            center_y_m=0.225,
            width_m=0.05,
            height_m=0.05,
        ),
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
            pose=Pose2D(x=-0.50, y=-0.075, yaw=0.0),
        )
        return cls(board=board, robot=robot, obstacles=default_obstacles())

    def reset(self) -> None:
        self.robot.reset(Pose2D(x=-0.50, y=-0.075, yaw=0.0))
        self.time_s = 0.0

    def step(self, dt_s: float) -> SensorSnapshot:
        self.robot.step(dt_s=dt_s, board=self.board, obstacles=self.obstacles)
        self.time_s += dt_s
        return self.read_sensors()

    def read_sensors(self) -> SensorSnapshot:
        return self.robot.read_sensors(board=self.board, obstacles=self.obstacles)

    def set_command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.robot.set_command(linear_m_s=linear_m_s, angular_rad_s=angular_rad_s)

