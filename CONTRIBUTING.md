# Contributing to cargo_bot

Thanks for your interest! **cargo_bot** is an autonomous indoor differential-drive cargo robot built on ROS 2 Humble (WSL2) and NVIDIA Isaac Sim 5.1 (Windows), using a Fast DDS Discovery Server. It is sim-first and organized into packages: `cargo_bot_description`, `cargo_bot_simulation`, `cargo_bot_navigation`, `cargo_bot_bringup`, and `cargo_bot_hardware`. Development progresses in **phases (Fase 0–7)**: DDS, URDF, Isaac scene, SLAM, Nav2, missions, hardware.

> This is primarily a solo / portfolio project. Issues and pull requests are very welcome, but review cadence may vary — please be patient.

## Prerequisites

See **[GETTING_STARTED.md](GETTING_STARTED.md)** for the full setup (WSL2 + ROS 2 Humble, Isaac Sim 5.1, Discovery Server, building the workspace). The phase roadmap lives in **[MASTER_PLAN.md](MASTER_PLAN.md)** and the per-phase technical guides in **[docs/](docs/)**.

## Branching model

We follow **GitHub Flow**:

1. Branch off `main`.
2. Open a pull request.
3. **Squash-merge** into `main`.
4. Delete the branch.

### Branch naming

Prefix branches by change type:

- `feat/` — new feature (e.g. `feat/fase4-nav2`)
- `fix/` — bug fix (e.g. `fix/scan-count-mismatch`)
- `docs/` — documentation
- `chore/` — tooling, deps, housekeeping
- `refactor/` — restructuring without behavior change

## Commit messages

We use **[Conventional Commits](https://www.conventionalcommits.org/)**. Format: `type(scope): description`.

Examples:

```
feat(navigation): add Nav2 bringup launch for Fase 4
fix(simulation): correct lidar scan plane to identity orientation
docs(master-plan): mark Fase 3 SLAM as complete
chore(bringup): add scan_angle_fixer node to install rules
```

## Pull request process

1. Build cleanly: `colcon build`.
2. Verify behavior where relevant — run Isaac Sim, check ROS topics / TF in RViz (`ros2 topic list`, `ros2 topic hz`, `ros2 topic echo`).
3. Fill out the PR template (type of change, how tested, checklist).
4. Use a Conventional Commit title — it becomes the squash-merge commit message.
5. Keep build artifacts out of the diff (`build/`, `install/`, `log/`).

## Docs-sync rule

If your change alters the plan, **update the docs in the same PR**. Any plan-changing decision must be reflected in `MASTER_PLAN.md` and the relevant `docs/FASE*_GUIA_*.md`. These are the single source of truth for the project's direction — keeping them in sync is required, not optional.

## Questions

Open an issue or start a discussion. Thanks for contributing!
