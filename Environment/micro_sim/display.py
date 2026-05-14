"""Terminal rendering helpers for the intersection MDP."""

from __future__ import annotations

from .world import Action, GridCell, IntersectionWorld, MdpAction, OrientedAction


ARROWS: dict[Action, str] = {
    Action.UP: "^",
    Action.DOWN: "v",
    Action.LEFT: "<",
    Action.RIGHT: ">",
}

OBSTACLE_GLYPH = "##"
GOAL_GLYPH = "GG"
START_GLYPH = "SS"
EMPTY_GLYPH = ".."


def print_layout(world: IntersectionWorld) -> None:
    """Print the static layout: start, goal, obstacles, free cells."""

    print(_separator(world))
    for row in range(world.rows):
        cells: list[str] = []
        for col in range(world.cols):
            cell = GridCell(row, col)
            if world.is_obstacle(cell):
                cells.append(OBSTACLE_GLYPH)
            elif cell == world.start:
                cells.append(START_GLYPH)
            elif cell == world.goal:
                cells.append(GOAL_GLYPH)
            else:
                cells.append(EMPTY_GLYPH)
        print(" ".join(cells))
    print(_separator(world))


def print_policy(
    world: IntersectionWorld,
    policy: dict[GridCell, MdpAction | None],
    robot_pos: GridCell | None = None,
) -> None:
    """Print the policy as a grid of arrows, plus obstacles and markers."""

    print(_separator(world))
    for row in range(world.rows):
        cells: list[str] = []
        for col in range(world.cols):
            cell = GridCell(row, col)
            if robot_pos is not None and cell == robot_pos:
                cells.append("RR")
            elif world.is_obstacle(cell):
                cells.append(OBSTACLE_GLYPH)
            elif cell == world.goal:
                cells.append(GOAL_GLYPH)
            elif cell in policy and policy[cell] is not None:
                act = policy[cell]
                if isinstance(act, OrientedAction):
                    cells.append(f" {act.value} ")
                else:
                    cells.append(f" {ARROWS[act]} ")
            else:
                cells.append(EMPTY_GLYPH)
        print(" ".join(cells))
    print(_separator(world))


def print_values(
    world: IntersectionWorld,
    values: dict[GridCell, float],
    width: int = 8,
) -> None:
    """Print V(s) as a numeric grid, with obstacles shown as ``####``."""

    print(_separator(world, cell_width=width + 1))
    for row in range(world.rows):
        cells: list[str] = []
        for col in range(world.cols):
            cell = GridCell(row, col)
            if world.is_obstacle(cell):
                cells.append("#" * width)
            elif cell in values:
                cells.append(f"{values[cell]:>{width}.1f}")
            else:
                cells.append("." * width)
        print(" ".join(cells))
    print(_separator(world, cell_width=width + 1))


def _separator(world: IntersectionWorld, cell_width: int = 3) -> str:
    return "-" * (world.cols * cell_width)
