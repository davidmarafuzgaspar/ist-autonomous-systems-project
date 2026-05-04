ARROWS = {
    'UP':    '↑',
    'DOWN':  '↓',
    'LEFT':  '←',
    'RIGHT': '→',
    'GOAL':  '🏁'
}

def print_grid(env, policy=None, robot_pos=None):
    print("\n" + "─" * (env.cols * 4))
    for r in range(env.rows):
        row_str = ""
        for c in range(env.cols):
            pos = (r, c)
            if pos == robot_pos:
                row_str += " 🤖"
            elif env.grid[r][c] == env.OBSTACLE:
                row_str += " █ "
            elif pos == env.goal:
                row_str += " 🏁"
            elif policy and pos in policy:
                row_str += f" {ARROWS.get(policy[pos], '?')} "
            else:
                row_str += " . "
        print(row_str)
    print("─" * (env.cols * 4) + "\n")