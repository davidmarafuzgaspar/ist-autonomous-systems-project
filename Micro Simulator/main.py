from environment.grid_world import GridWorld
from mdp_agent.value_iteration    import ValueIteration
from utils.visualizer       import print_grid

def main():
    # 1. Create environment
    env = GridWorld(rows=5, cols=5, random_map=True)

    print("=== INITIAL GRID ===")
    print_grid(env)

    # 2. Solve MDP
    solver = ValueIteration(env, gamma=0.85, theta=1e-3)
    V, policy = solver.solve()

    # 3. Show optimal policy
    print("=== OPTIMAL POLICY ===")
    print_grid(env, policy=policy)

    # 4. Execute policy step by step
    print("=== ROBOT FOLLOWING POLICY ===")
    state = env.reset()
    done  = False
    steps = 0

    while not done and steps < 50:
        action = policy.get(state, 'UP')
        state, reward, done = env.apply_action(action)
        steps += 1
        print(f"Step {steps:2d} | Action: {action:5s} | "
              f"State: {state} | Reward: {reward}")
        print_grid(env, policy=policy, robot_pos=state)

    if done:
        print(f" Goal reached in {steps} steps!")
    else:
        print(" Did not reach goal — check your reward function.")

if __name__ == "__main__":
    main()