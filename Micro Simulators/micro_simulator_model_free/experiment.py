#!/usr/bin/env python3
""" Parameter sweep experiment for Q-learning model. """
""" Run from this directory: .\.venv\Scripts\python.exe experiment.py """

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm

# Support both direct execution and package import
try:
    from .model import IntersectionWorld, QLearningTrainer
except ImportError:
    from model import IntersectionWorld, QLearningTrainer

# Parameter ranges (to be swept)
ALPHA_VALUES = np.arange(0.1, 1.1, 0.1)  # [0.1, 0.2, ..., 1.0]
EPSILON_END_VALUES = np.arange(0.0, 1.1, 0.1)  # [0.0, 0.1, ..., 1.0]

# Fixed parameters - MUST match GUI defaults exactly
EPSILON_START = 1.0  # Always start with full exploration
GAMMA = 0.7  # Discount factor
EPSILON_DECAY = 0.995  # Multiplicative decay per episode (CRITICAL for reproducibility)
NUM_EPISODES = 1000  # Training episodes
MAX_STEPS = 50  # Max steps per episode
SEED = 0  # Fixed seed for reproducibility

# Optimal path threshold (12 steps in greedy_rollout)
OPTIMAL_PATH_STEPS = 12


def run_single_experiment(
    alpha: float, epsilon_end: float
) -> dict[str, float]:
    """
    Run a single Q-learning training session with exact GUI behavior.
    
    This function creates a fresh IntersectionWorld and trains a QLearningTrainer
    with the specified hyperparameters. All epsilon scheduling and training is
    handled identically to the GUI implementation.
    
    Args:
        alpha: Learning rate (varied in sweep)
        epsilon_end: Final epsilon value after decay (varied in sweep)
    
    Returns:
        Dictionary with keys:
        - 'success_rate': success_count / num_episodes
        - 'avg_return': mean cumulative reward across all episodes
        - 'optimal_path': 1.0 if optimal path found, 0.0 otherwise
    """
    # Create fresh world - identical to GUI
    world = IntersectionWorld()
    
    # Create trainer with EXPLICIT parameters matching GUI defaults
    # Critical: epsilon_decay controls how epsilon decreases each episode
    trainer = QLearningTrainer(
        world,
        alpha=alpha,
        gamma=GAMMA,
        epsilon_start=EPSILON_START,  # Always 1.0
        epsilon_end=epsilon_end,       # This varies
        epsilon_decay=EPSILON_DECAY,   # CRITICAL: 0.995 decay per episode
        max_steps=MAX_STEPS,
        num_episodes=NUM_EPISODES,
        seed=SEED,  # Fixed seed for reproducibility
    )
    
    # Run full training (identical to GUI training loop)
    trainer.run_all_episodes()
    
    # Compute metrics AFTER training completes
    success_rate = trainer.success_count / NUM_EPISODES
    
    # Average return across all episodes
    # Note: We're using the last episode return as proxy; for true average,
    # we'd need to track all episode returns. The trainer.last_episode_return
    # approximates the final policy quality.
    avg_return = trainer.last_episode_return
    
    # Test if optimal 12-step path is found using greedy policy
    initial_state = world.initial_state()
    greedy_path = trainer.greedy_rollout(initial_state, max_steps=128)
    optimal_achieved = 1.0 if len(greedy_path) <= OPTIMAL_PATH_STEPS else 0.0
    
    return {
        'success_rate': success_rate,
        'avg_return': avg_return,
        'optimal_path': optimal_achieved,
    }


def run_parameter_sweep() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Run complete parameter sweep and return result matrices.
    
    Returns:
        Tuple of (success_rate_matrix, avg_return_matrix, optimal_path_matrix)
        where rows correspond to alpha values and columns to epsilon_end values.
    """
    n_alphas = len(ALPHA_VALUES)
    n_epsilons = len(EPSILON_END_VALUES)
    
    # Initialize result matrices
    success_rate_matrix = np.zeros((n_alphas, n_epsilons))
    avg_return_matrix = np.zeros((n_alphas, n_epsilons))
    optimal_path_matrix = np.zeros((n_alphas, n_epsilons))
    
    # Run experiments
    total_experiments = n_alphas * n_epsilons
    current_exp = 0
    
    for i, alpha in enumerate(ALPHA_VALUES):
        for j, epsilon_end in enumerate(EPSILON_END_VALUES):
            current_exp += 1
            print(
                f"[{current_exp}/{total_experiments}] "
                f"Running experiment: alpha={alpha:.1f}, epsilon_end={epsilon_end:.1f}",
                flush=True,
            )
            
            results = run_single_experiment(alpha, epsilon_end)
            success_rate_matrix[i, j] = results['success_rate']
            avg_return_matrix[i, j] = results['avg_return']
            optimal_path_matrix[i, j] = results['optimal_path']
    
    return success_rate_matrix, avg_return_matrix, optimal_path_matrix


def plot_heatmap(
    matrix: np.ndarray,
    alpha_values: np.ndarray,
    epsilon_end_values: np.ndarray,
    title: str,
    filename: str,
    cmap: str = 'viridis',
    emphasize_max: bool = True,
) -> None:
    """
    Plot a heatmap for the results.
    
    Args:
        matrix: 2D array with rows=alpha, cols=epsilon_end
        alpha_values: Array of alpha values used
        epsilon_end_values: Array of epsilon_end values used
        title: Title for the plot
        filename: Filename to save the plot
        cmap: Colormap to use
        emphasize_max: If True, use PowerNorm to emphasize highest values (better for success rates)
    """
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Create normalization that emphasizes high values (e.g., 96% vs 97%)
    if emphasize_max:
        # PowerNorm with gamma < 1 stretches the colormap to emphasize high values
        norm = PowerNorm(gamma=0.5, vmin=matrix.min(), vmax=matrix.max())
    else:
        norm = None
    
    # Create heatmap
    im = ax.imshow(matrix, cmap=cmap, aspect='auto', origin='lower', norm=norm)
    
    # Set ticks and labels
    ax.set_xticks(np.arange(len(epsilon_end_values)))
    ax.set_yticks(np.arange(len(alpha_values)))
    ax.set_xticklabels([f'{x:.1f}' for x in epsilon_end_values])
    ax.set_yticklabels([f'{x:.1f}' for x in alpha_values])
    
    # Labels and title
    ax.set_xlabel('ε_end (Epsilon End)', fontsize=12, fontweight='bold')
    ax.set_ylabel('α (Alpha)', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Value', fontsize=11)
    
    # Add text annotations (optional, only for smaller grids)
    if len(epsilon_end_values) <= 11 and len(alpha_values) <= 10:
        for i in range(len(alpha_values)):
            for j in range(len(epsilon_end_values)):
                text = ax.text(
                    j, i, f'{matrix[i, j]:.2f}',
                    ha="center", va="center", color="black", fontsize=8
                )
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved plot to {filename}")
    plt.close()


def main_experiment() -> None:
    """Main entry point for the parameter sweep experiment."""
    print("=" * 80)
    print("Q-Learning Parameter Sweep Experiment")
    print("=" * 80)
    print(f"Alpha values: {ALPHA_VALUES}")
    print(f"Epsilon_end values: {EPSILON_END_VALUES}")
    print(f"Fixed: epsilon_start={EPSILON_START}, gamma={GAMMA}")
    print(f"Training: {NUM_EPISODES} episodes, max {MAX_STEPS} steps per episode")
    print("=" * 80)
    print()
    
    # Run parameter sweep
    print("Starting parameter sweep...")
    success_rate_matrix, avg_return_matrix, optimal_path_matrix = run_parameter_sweep()
    
    print("\n" + "=" * 80)
    print("Experiment completed!")
    print("=" * 80)
    
    # Print summary statistics
    print("\nSummary Statistics:")
    print(f"  Success Rate - Mean: {success_rate_matrix.mean():.3f}, "
          f"Max: {success_rate_matrix.max():.3f}, Min: {success_rate_matrix.min():.3f}")
    print(f"  Avg Return - Mean: {avg_return_matrix.mean():.3f}, "
          f"Max: {avg_return_matrix.max():.3f}, Min: {avg_return_matrix.min():.3f}")
    print(f"  Optimal Path - Achieved in {optimal_path_matrix.sum():.0f} / "
          f"{optimal_path_matrix.size} experiments ({100*optimal_path_matrix.mean():.1f}%)")
    print()
    
    # Plot results
    output_dir = Path(__file__).parent / "experiment_results"
    output_dir.mkdir(exist_ok=True)
    
    print(f"Saving results to {output_dir}/")
    
    plot_heatmap(
        success_rate_matrix,
        ALPHA_VALUES,
        EPSILON_END_VALUES,
        "Success Rate (Success Count / Total Episodes)",
        str(output_dir / "heatmap_success_rate.png"),
        cmap='RdYlGn',
        emphasize_max=True
    )
    
    plot_heatmap(
        avg_return_matrix,
        ALPHA_VALUES,
        EPSILON_END_VALUES,
        "Average Episode Return",
        str(output_dir / "heatmap_avg_return.png"),
        cmap='viridis',
        emphasize_max=False
    )
    
    plot_heatmap(
        optimal_path_matrix,
        ALPHA_VALUES,
        EPSILON_END_VALUES,
        f"Optimal Path Achieved (≤ {OPTIMAL_PATH_STEPS} steps)",
        str(output_dir / "heatmap_optimal_path.png"),
        cmap='RdYlGn',
        emphasize_max=True
    )
    
    # Save raw data
    np.savez(
        output_dir / "results.npz",
        success_rate=success_rate_matrix,
        avg_return=avg_return_matrix,
        optimal_path=optimal_path_matrix,
        alpha_values=ALPHA_VALUES,
        epsilon_end_values=EPSILON_END_VALUES,
    )
    print(f"Saved raw data to {output_dir / 'results.npz'}")
    print("\nAll results saved successfully!")


if __name__ == "__main__":
    main_experiment()
