from dataclasses import dataclass

MIN_LINES = 3
MAX_LINES = 9
MIN_COLUMNS = 3
MAX_COLUMNS = 9

DEFAULT_LINES = 3
DEFAULT_COLUMNS = 5


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
        return (
            (crosses - 1) * spacing_m / 2.0
            + spacing_m / 2.0
            + outer_extension_m
        )

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
