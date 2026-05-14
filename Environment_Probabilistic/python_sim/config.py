from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BoardConfig:
    crosses_per_axis: int = 5
    spacing_m: float = 0.30
    line_width_m: float = 0.07
    outward_margin_m: float = 0.0
    outer_extension_m: float = 0.15
    margin_m: float = 0.12

    @property
    def line_extent_m(self) -> float:
        return (
            (self.crosses_per_axis - 1) * self.spacing_m / 2.0
            + self.spacing_m / 2.0
            + self.outer_extension_m
        )

    @property
    def footprint_m(self) -> float:
        return 2.0 * self.line_extent_m + 2.0 * self.outward_margin_m

    @property
    def half_extent_m(self) -> float:
        return self.footprint_m / 2.0


@dataclass(frozen=True)
class RobotConfig:
    diameter_m: float = 0.17
    line_sensor_count: int = 5
    line_sensor_front_offset_m: float = 0.025
    line_sensor_spacing_m: float = 0.025
    obstacle_sensor_max_range_m: float = 0.07
    obstacle_sensor_lateral_offset_m: float = 0.028
    obstacle_sensor_front_clearance_m: float = 0.008
    camera_local_x_m: float = 0.055
    camera_local_y_m: float = 0.030
    camera_yaw_offset_rad: float = math.radians(45.0)
    camera_fov_rad: float = math.radians(75.0)
    camera_max_range_m: float = 0.18
    max_linear_speed_m_s: float = 0.25
    max_angular_speed_rad_s: float = 2.5
    line_black_value: int = 200
    line_white_value: int = 900

    @property
    def radius_m(self) -> float:
        return self.diameter_m / 2.0

    @property
    def line_sensor_local_x_m(self) -> float:
        return self.radius_m - self.line_sensor_front_offset_m

    @property
    def obstacle_sensor_local_x_m(self) -> float:
        return self.radius_m - self.obstacle_sensor_front_clearance_m

