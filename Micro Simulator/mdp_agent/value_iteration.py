import numpy as np

class ValueIteration:

    def __init__(self, env, gamma=0.85, theta=1e-3):
        self.env   = env
        self.gamma = gamma
        self.theta = theta

    def _get_transition_prob(self, action):
        """
        Returns probability distribution over actual actions.
        Mirrors the drift model in grid_world._apply_drift().
        """
        actions = list(self.env.ACTIONS.keys())
        idx     = actions.index(action)
        return {
            action:                  0.70,
            actions[(idx + 1) % 4]: 0.15,   # drift right
            actions[(idx - 1) % 4]: 0.15,   # drift left
        }

    def _get_next_state(self, state, action):
        """
        Deterministically compute next state for a given action.
        No randomness — pure physics.
        """
        dr, dc = self.env.ACTIONS[action]
        r, c   = state
        new_r  = r + dr
        new_c  = c + dc

        if self.env._is_valid(new_r, new_c):
            return (new_r, new_c), False    # (new_state, hit_wall)
        else:
            return state, True              # stayed in place, hit wall

    def _expected_value(self, state, action, V):
        """
        Compute expected value analytically over all drift outcomes.
        E[V] = sum of P(actual|intended) * (R + gamma * V(s'))
        No randomness — pure math.
        """
        total = 0.0
        for actual_action, prob in self._get_transition_prob(action).items():
            s_next, hit_wall = self._get_next_state(state, actual_action)
            reward = self.env._get_reward(state, s_next, hit_wall)
            total += prob * (reward + self.gamma * V.get(s_next, 0.0))
        return total

    def solve(self):
        states  = self.env.get_all_states()
        actions = list(self.env.ACTIONS.keys())

        V        = {s: 0.0 for s in states}
        MAX_ITER = 1000          # ← 1000 is plenty, not 100000
        iteration = 0

        while iteration < MAX_ITER:
            delta = 0

            for s in states:
                if s == self.env.goal:
                    continue

                # ── Bellman update using expected value ──
                best   = max(self._expected_value(s, a, V) for a in actions)
                delta  = max(delta, abs(V[s] - best))
                V[s]   = best

            iteration += 1
            print(f"Iteration {iteration} | delta = {delta:.6f}")

            if delta < self.theta:
                print(f"Converged after {iteration} iterations")
                break

        else:
            # ← only prints once if MAX_ITER is reached without converging
            print(f"Warning: did not converge after {MAX_ITER} iterations.")

        # ── Extract policy using expected value too ──
        policy = {}
        for s in states:
            if s == self.env.goal:
                policy[s] = 'GOAL'
                continue

            # ← also uses _expected_value, no randomness
            policy[s] = max(
                actions,
                key=lambda a: self._expected_value(s, a, V)
            )

        return V, policy