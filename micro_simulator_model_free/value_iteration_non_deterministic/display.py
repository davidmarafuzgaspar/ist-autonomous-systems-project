"""Terminal rendering of grid layout, per-cell max V, and aggregated policy."""

from __future__ import annotations

from .world import GridAction, GridCell, Heading, IntersectionWorld

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

ACTION_GLYPHS: dict[GridAction, str] = {
    GridAction.STRAIGHT: "S",
    GridAction.TURN_RIGHT: "R",
    GridAction.TURN_LEFT: "L",
    GridAction.TURN_AROUND: "A",
}


def policy_glyph(action: GridAction, draw_heading: Heading, world: IntersectionWorld) -> str:
    if action == GridAction.STRAIGHT:
        move_h = world.movement_heading_for_action(draw_heading, action)
        return HEADING_ARROWS[move_h]
    return ACTION_GLYPHS[action]


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
    print(
        f"start=({world.start.row},{world.start.col},{world.start_heading.name})  "
        f"goal=({world.goal.row},{world.goal.col})"
    )


def print_policy(
    world: IntersectionWorld,
    policy: dict[GridCell, GridAction | None],
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
                cells.append(f" {policy_glyph(act, h, world)} ")
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
