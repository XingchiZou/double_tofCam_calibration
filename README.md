# dual_tofCam_calibration

ROS 1 Noetic package for dual-camera pivot calibration using pure rotation and PCL ICP.

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

- `src/dual_cam_pivot_calib/` — C++ calibration node
- `src/dual_cam_pivot_calib/scripts/synthetic_pivot_publisher.py` — synthetic test data
- `scripts/demo.sh` — automated workflow demo
- `scripts/build.sh` — catkin build helper

## ROS interfaces

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/cam1/points` | `sensor_msgs/PointCloud2` | Camera 1 point cloud |
| `/cam2/points` | `sensor_msgs/PointCloud2` | Camera 2 point cloud |
| `~/set_reference` | `std_srvs/Trigger` | Capture reference frame |
| `~/add_pivot_sample` | `std_srvs/Trigger` | Add ICP motion sample |
| `~/calibrate` | `std_srvs/Trigger` | Run calibration (needs ≥2 samples) |

See `Prompt.md` for the full algorithm specification.
