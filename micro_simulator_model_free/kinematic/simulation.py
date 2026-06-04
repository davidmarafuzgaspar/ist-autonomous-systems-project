from __future__ import annotations

from dataclasses import dataclass, field

from .board import CrossBoard
from .config import BoardConfig, RobotConfig
from .obstacles import RectangleObstacle, snap_obstacles_to_board
from .robot import AlphaBot2Robot, Pose2D, SensorSnapshot
from .world_setup import WorldSetup


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
