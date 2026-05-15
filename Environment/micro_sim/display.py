"""Terminal output for layout, values and aggregated policy."""

from __future__ import annotations

from .world import GridCell, Heading, IntersectionWorld, OrientedAction

OBSTACLE_GLYPH = "##"
GOAL_GLYPH = "GG"
START_GLYPH = "SS"
EMPTY_GLYPH = ".."

HEADING_ARROWS: dict[Heading, str] = {
    Heading.N: "^",
    Heading.E: ">",
    Heading.S: "v",
    Heading.W: "<",
}


def oriented_policy_glyph(action: OrientedAction, draw_heading: Heading) -> str:
    if action == OrientedAction.FORWARD:
        return HEADING_ARROWS[draw_heading]
    if action == OrientedAction.TURN_LEFT:
        return "\u21ba"
    if action == OrientedAction.TURN_RIGHT:
        return "\u21bb"
    return action.value


def print_layout(world: IntersectionWorld) -> None:
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
    policy: dict[GridCell, OrientedAction | None],
    draw_heading: dict[GridCell, Heading] | None = None,
) -> None:
    print(_separator(world))
    for row in range(world.rows):
        cells: list[str] = []
        for col in range(world.cols):
            cell = GridCell(row, col)
            if world.is_obstacle(cell):
                cells.append(OBSTACLE_GLYPH)
            elif cell == world.goal:
                cells.append(GOAL_GLYPH)
            elif cell in policy and policy[cell] is not None:
                act = policy[cell]
                assert act is not None
                h = (draw_heading or {}).get(cell, Heading.N)
                cells.append(f" {oriented_policy_glyph(act, h)} ")
            else:
                cells.append(EMPTY_GLYPH)
        print(" ".join(cells))
    print(_separator(world))


def print_values(world: IntersectionWorld, values: dict[GridCell, float], width: int = 8) -> None:
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
