# Diffusion-Planner Project Instructions

This document provides foundational mandates for the Diffusion-Planner project, covering architecture, development workflows, and coding standards.

## Project Identity & Architecture

Diffusion-Planner is a diffusion-based trajectory planning system for autonomous driving. It aims to generate safe, comfortable, and goal-directed trajectories in complex environments.

### Module Overview

- **`diffusion_planner/`**: Core diffusion model implementation, training scripts (SFT), and utilities.
- **`diffusion_planner_ros/`**: ROS 2 wrapper for deploying the planner in a ROS environment.
- **`preference_optimization/`**: Implements Direct Preference Optimization (DPO) to refine the planner using human or rule-based trajectory preferences.
- **`rlvr/`**: Reinforcement Learning with Verifiable Rewards. Implements Group Relative Policy Optimization (GRPO) for self-improvement and RL fine-tuning.
- **`scenario_generation/`**: Tools for generating, replaying, and simulating driving scenarios from NPZ data.
- **`scene_search/`**: Tools for indexing and searching for specific scenarios within large datasets.
- **`cpp_tools/`**: Performance-critical C++ components for data processing and ROS integration.
- **`docs/`**: Design documents and framework specifications.

## Core Workflows

### 1. Data Preparation
- **Raw Data**: Assumes `rosbag2` format stored in a specific directory structure (see `README.md`).
- **Conversion**: Use `ros_scripts/parse_rosbag_for_directory.py` to convert rosbags to the internal `*.npz` format.
- **Path Indexing**: Generate `path_list.json` using `diffusion_planner/util_scripts/create_train_set_path.py`.

### 2. Training Pipeline
- **SFT (Supervised Fine-Tuning)**: Initial training on logged human data using `diffusion_planner/train_run.sh`.
- **DPO (Preference Optimization)**: Refine models using `preference_optimization/train_dpo.py`. Supports Gradio-based web UI for human annotation.
- **GRPO (Reinforcement Learning)**: RL-based fine-tuning using `rlvr/train_grpo.py`. Focuses on verifiable rewards (safety, progress, etc.).
- **LoRA**: Preference optimization and RL often use LoRA (Low-Rank Adaptation) for efficient fine-tuning.

### 3. Evaluation & Visualization
- **Trajectory Ranker GUI**: Use `rlvr/trajectory_ranker_gui.py` to inspect rewards and compare trajectories.
- **Annotation GUI**: Use `preference_optimization/train_dpo.py --preference_mode gui` for manual preference collection.
- **Closed-Loop Simulation**: Use `scenario_generation/simulate.py` for evaluating planner performance in simulation.

## Engineering Standards

### Coding Style & Linting
- **Python**: Follow PEP 8. Use **type hints** for all function signatures.
- **Linter**: `ruff` is used for linting and formatting. 
  - Line length: 100 characters.
  - Configuration is in `pyproject.toml`.
  - To lint: `ruff check .`
- **Docstrings**: Provide comprehensive docstrings for all modules, classes, and public functions.

### Testing
- **Framework**: `pytest`.
- **Conventions**:
  - Unit tests for reward components and utilities.
  - Integration tests for training loops and sampler logic.
  - Run tests with `pytest`.
  - Use `-m "not benchmark"` to skip performance-heavy tests.

### Environment & Dependencies
- **Dependency Management**: Standard `pip` with `requirements.txt` or `uv`.
- **Hardware**: NVIDIA GPU with CUDA is required for training and high-frequency inference.
- **Key Libraries**: PyTorch, PyTorch Lightning, PEFT (for LoRA), Gradio, ROS 2 (Humble).

## Shared Conventions

- **File Formats**: `*.npz` is the primary format for scene data. `*.pth` or LoRA adapter directories are used for model checkpoints.
- **Coordinate Frames**: Most planners operate in an ego-relative frame at t=0, but world-frame conversions are common in visualization.
- **Commit Style**: Use clear, concise commit messages focusing on the "why". Gather info with `git status` and `git diff` before committing.
