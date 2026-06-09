# dual_tofCam_calibration

ROS 1 Noetic package for dual-camera full calibration using pivot rotation, straight-line motion, and PCL ICP.

## Prerequisites

- Docker with Compose plugin
- Ubuntu host (tested on 24.04; ROS Noetic runs inside Docker on Focal)

## Quick start

Build the development image and compile the package:

```bash
docker compose build
docker compose run --rm ros-dev ./scripts/build.sh
```

Run the end-to-end calibration demo (roscore + synthetic clouds + calibration services):

```bash
docker compose run --rm ros-dev ./scripts/demo.sh
```

## Package layout

- `src/dual_cam_full_calib/` — C++ calibration node
- `src/dual_cam_full_calib/config/config.yaml` — configurable parameters (topics, ICP, thresholds)
- `src/dual_cam_full_calib/launch/dual_cam_full_calib.launch` — launch file
- `src/dual_cam_full_calib/scripts/synthetic_full_publisher.py` — synthetic test data
- `scripts/demo.sh` — automated workflow demo
- `scripts/build.sh` — catkin build helper

## ROS interfaces

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/cam1/points` | `sensor_msgs/PointCloud2` | Camera 1 point cloud |
| `/cam2/points` | `sensor_msgs/PointCloud2` | Camera 2 point cloud |
| `~/set_reference` | `std_srvs/Trigger` | Capture reference frame |
| `~/add_pivot_sample` | `std_srvs/Trigger` | Add rotation ICP motion sample |
| `~/add_straight_sample` | `std_srvs/Trigger` | Add straight-line ICP motion sample (Cam1 only) |
| `~/calibrate` | `std_srvs/Trigger` | Run 6-phase calibration (needs ≥2 pivot + ≥1 straight) |

## Calibration output

The `calibrate` service prints to terminal:
- **R_align1 / R_align2** — alignment rotation matrices (mechanical mounting correction)
- **r1 / r2** — horizontal lever arms for each camera
- **X (T_C1←C2)** — extrinsic transform from Camera 2 to Camera 1
- **ψ** — yaw correction angle (degrees)

See `Prompt.md` for the full algorithm specification.
