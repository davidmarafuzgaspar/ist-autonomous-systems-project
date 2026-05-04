import numpy as np
import random

class GridWorld:
    """
    Simulated environment.
    Later this gets replaced by ros_world.py
    with real sensor data — MDP agent never changes.
    """

    FREE     = 0
    OBSTACLE = 1
    GOAL     = 2
    START    = 3

    ACTIONS = {
        'UP':    (-1,  0),
        'DOWN':  (1,  0),
        'LEFT':  ( 0, -1),
        'RIGHT': ( 0,  1)
    }

    def __init__(self, rows=5, cols=5, random_map=True):
        self.rows = rows
        self.cols = cols
        self.grid = self._generate_map() if random_map else self._default_map()
        self.start = self._find_cell(self.START)
        self.goal  = self._find_cell(self.GOAL)
        self.robot_pos = self.start

    # ── Map generation ──────────────────────────────────────

    def _generate_map(self):
        """Random maze with guaranteed start, goal and free cells."""
        grid = np.zeros((self.rows, self.cols), dtype=int)

        # Random obstacles (~20% of cells)
        for r in range(self.rows):
            for c in range(self.cols):
                if random.random() < 0.2:
                    grid[r][c] = self.OBSTACLE

        # Always place start top-left, goal bottom-right
        grid[0][0] = self.START
        grid[self.rows-1][self.cols-1] = self.GOAL

        return grid

    def _default_map(self):
        return np.array([
            [3, 0, 0, 0, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 1],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 2]
        ])

    def _find_cell(self, cell_type):
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == cell_type:
                    return (r, c)

    # ── Interface (this is what MDP Agent calls) ─────────────

    def get_state(self):
        """
        Returns current robot position.
        ROS2 version: this will return position from ArUCo detection.
        """
        return self.robot_pos

    def get_all_states(self):
        """All valid (non-obstacle) states."""
        return [
            (r, c)
            for r in range(self.rows)
            for c in range(self.cols)
            if self.grid[r][c] != self.OBSTACLE
        ]

    def apply_action(self, action):
        """
        Move robot, apply drift stochasticity.
        ROS2 version: this will publish to /cmd_vel.
        Returns: (new_state, reward, done)
        """
        actual_action = self._apply_drift(action)
        dr, dc = self.ACTIONS[actual_action]
        r, c = self.robot_pos
        new_r = r + dr
        new_c = c + dc

        old_pos = self.robot_pos 
        hit_wall = False
        
        # Stay in place if hitting wall or obstacle
        if self._is_valid(new_r, new_c):
            self.robot_pos = (new_r, new_c)

        else:
         hit_wall = True

        reward = self._get_reward(old_pos, self.robot_pos, hit_wall)
        done   = (self.robot_pos == self.goal)

        return self.robot_pos, reward, done

    def is_obstacle(self, r, c):
        """
        Check if cell is blocked.
        ROS2 version: this reads from proximity sensor topic.
        """
        if not self._is_valid(r, c):
            return True
        return self.grid[r][c] == self.OBSTACLE

    def reset(self):
        self.robot_pos = self.start
        return self.robot_pos

    # ── Internal helpers ─────────────────────────────────────

    def _is_valid(self, r, c):
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return False                           # out of bounds
        if self.grid[r][c] == self.OBSTACLE:
            return False                           # obstacle cell
        return True

    def _apply_drift(self, action):
        """
        Simulates the drift you observed in the real robot.
        70% goes intended direction, 15% drifts each side.
        """
        actions = list(self.ACTIONS.keys())
        idx = actions.index(action)

        drift_probs = {
            action:                         0.70,
            actions[(idx + 1) % 4]:         0.15,  # drift right
            actions[(idx - 1) % 4]:         0.15,  # drift left
        }
        return random.choices(
            list(drift_probs.keys()),
            weights=list(drift_probs.values())
        )[0]

    def _get_reward(self, old_pos, new_pos, hit_wall=False):
        """
        Reward function.
        ROS2 version: ArUCo seen = goal reached, 
                    proximity triggered = obstacle/wall penalty.
        """
        # ── Terminal state ────────────────────────────────────────
        if new_pos == self.goal:
            return +10000          # large positive — this is what we want

        # ── Collision (wall or obstacle — same thing) ─────────────
        elif hit_wall:
            return -500            # severe — robot must avoid collisions

        # ── Movement penalties ────────────────────────────────────
        else:
            old_dist = (abs(old_pos[0] - self.goal[0]) + 
                        abs(old_pos[1] - self.goal[1]))
            new_dist = (abs(new_pos[0] - self.goal[0]) + 
                        abs(new_pos[1] - self.goal[1]))

            if new_dist > old_dist:
                return -100        # moved away from goal → punish hard
            else:
                return -1          # moved toward goal → almost free