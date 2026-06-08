# AGENTS.md

## Cursor Cloud specific instructions

### Product overview

This repo implements `dual_cam_pivot_calib`, a ROS 1 Noetic C++ node for dual ToF/camera pivot calibration via pure rotation and PCL ICP. The host VM runs Ubuntu 24.04; **ROS Noetic runs inside Docker** (Focal-based `ros:noetic-ros-base-focal` image) because Noetic is not supported natively on 24.04.

### Services

| Service | Required | Notes |
|---------|----------|-------|
| Docker daemon | Yes | Must be running before any build/run. In Cloud VMs, start with `sudo dockerd` if not already up (see Dockerfile setup in this branch). |
| `ros-dev` (docker compose) | Yes | Provides ROS Noetic, PCL, Eigen, catkin workspace at `/catkin_ws`. |
| `roscore` | Yes (runtime) | Started automatically by `scripts/demo.sh`. |
| Synthetic publisher / real cameras | Yes (runtime) | Demo uses `synthetic_pivot_publisher.py`; real hardware publishes to `/cam1/points` and `/cam2/points`. |

### Common commands

All commands run from repo root:

```bash
# Build image + compile package
docker compose build
docker compose run --rm ros-dev ./scripts/build.sh

# End-to-end demo (roscore → synthetic clouds → set_reference → add_pivot_sample ×3 → calibrate)
docker compose run --rm ros-dev ./scripts/demo.sh

# Interactive dev shell with ROS env sourced
docker compose run --rm ros-dev bash
```

Inside the container, ROS is sourced via `/entrypoint.sh`. Rebuild after C++ changes with `./scripts/build.sh`.

### Lint / test

There is no separate linter configured. Validation is:

1. `docker compose build` (compiles C++14 node with `-DCMAKE_BUILD_TYPE=Release`)
2. `docker compose run --rm ros-dev ./scripts/demo.sh` (all services return `success: True`)

### Gotchas

- **Docker on Cloud VM**: Uses `fuse-overlayfs` storage driver and `iptables-legacy`. If `docker compose` fails with permission errors, ensure `dockerd` is running.
- **Volume mounts**: `src/` is bind-mounted into the container; C++ edits on the host require `./scripts/build.sh` inside the container to recompile.
- **No native ROS**: Do not expect `roscore` or `catkin_make` on the host PATH; always use `docker compose run`.
- **Original spec**: Algorithm requirements live in `Prompt.md` (Chinese). The implemented node follows that spec.
