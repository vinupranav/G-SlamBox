<div align="center">

# G-SlamBox

### GPU-Accelerated 2D SLAM for Embedded Robotics

[![CUDA](https://img.shields.io/badge/CUDA-12.6%2B-76B900?style=flat&logo=nvidia)](https://developer.nvidia.com/cuda-toolkit)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?style=flat&logo=ros)](https://docs.ros.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin-76B900?style=flat&logo=nvidia)](https://developer.nvidia.com/embedded-computing)
[![License](https://img.shields.io/badge/License-Coming%20Soon-lightgrey?style=flat)]()

**G-SlamBox replaces CPU scan matching with custom CUDA kernels, delivering 7.5× faster SLAM at higher accuracy on embedded edge devices.**

*Source code will be released upon publication of the accompanying paper.*

---

[Key Results](#key-results) · [Architecture](#architecture) · [Benchmarks](#benchmarks) · [Getting Started](#getting-started) · [Citation](#citation)

</div>

---

## The Problem

Accurate real-time localisation on embedded edge devices is a fundamental bottleneck for autonomous mobile robots. CPU-based correlative scan matching in systems like [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) must trade search resolution for speed — searching fewer candidate poses to meet timing constraints, which directly reduces localisation accuracy. On resource-constrained platforms like the NVIDIA Jetson Orin Nano, this tradeoff becomes critical: the CPU cannot search a dense pose space fast enough, forcing coarser resolution that misses the optimal alignment.

## The Solution

G-SlamBox moves the correlative scan matching pipeline entirely to the GPU using **9 custom CUDA kernels**, enabling a 15× denser search space at 7.5× faster execution. The system is a drop-in replacement for slam_toolbox's scan matcher, maintaining full compatibility with the ROS 2 navigation stack.

## Key Results

| Metric | G-SlamBox (GPU) | CPU slam_toolbox | Improvement |
|---|---|---|---|
| **Scan match time** | 10.65 ms | 79.37 ms | **7.5× faster** |
| **Candidate poses per scan** | 314,721 | 20,956 | **15× denser** |
| **Position drift** | 9.8 mm | 15.1 mm | **35% lower** |
| **Angular drift** | 0.28° | 6.03° | **21× lower** |
| **CPU utilisation** | 10% | 24.7% | **60% reduction** |
| **System power** | 5.23 W | 5.93 W | **12% lower** |

> Evaluated on a real tracked robot with YDLidar TG50 (12Hz) on Jetson Orin Nano Super (sm_87, 7.4GB shared RAM).

### vs NVIDIA cuVSLAM

On the same hardware and trajectory, NVIDIA's visual SLAM drifted **3.12 m** on a closed loop where G-SlamBox drifted **9.8 mm** — a 318× improvement in return-to-origin accuracy.

### Intel Research Lab Benchmark

Validated on the [Intel Research Lab dataset](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php) (13,631 laser scans, 28m × 28m building):

| | G-SlamBox | CPU slam_toolbox |
|---|---|---|
| Avg time per scan | **5.06 ms** | 17.07 ms |
| Scans processed | 2,356 | 1,301 |
| Search poses | 314,721 | 20,956 |

GPU searches **15× more poses in 3.4× less time**.

---

## Architecture

G-SlamBox is structured as a 5-layer architecture that cleanly separates CUDA kernels from ROS 2 integration:

```
┌─────────────────────────────────────────────────────┐
│                  ROS 2 Navigation Stack             │
│            (Nav2, EKF, nvblox, TF, map)             │
├─────────────────────────────────────────────────────┤
│  g_toolbox_common    │  Patched slam_toolbox_common │
│                      │  Initialises GPU bridge      │
├──────────────────────┤──────────────────────────────┤
│  g_kartoSlamToolbox  │  Patched Mapper.cpp          │
│                      │  Intercepts MatchScan calls  │
├──────────────────────┤──────────────────────────────┤
│  g_slam_karto_bridge │  Type conversion layer       │
│                      │  karto ↔ GPU types           │
├──────────────────────┤──────────────────────────────┤
│  g_slam_core         │  Host orchestration          │
│                      │  Memory management, uploads  │
├──────────────────────┤──────────────────────────────┤
│  g_slam_cuda         │  9 CUDA kernels              │
│                      │  All GPU computation          │
└─────────────────────────────────────────────────────┘
```

### 9 CUDA Kernels

| Kernel | Purpose |
|---|---|
| `clear_grid` | Reset correlation grid |
| `populate_grid` | Gaussian smearing of occupied cells |
| `compute_offsets` | Precompute scan point grid offsets per angle |
| `scan_match` | Parallel correlative scan matching with inline per-XY reduction |
| `reduce_best_pose` | Multi-stage GPU reduction to find optimal pose |
| `positional_covariance` | Response surface covariance estimation |
| `scan_polar_to_cartesian` | Scan coordinate conversion |
| `raytrace` | Grid raytracing for map updates |
| `compute_edge_residuals` | Pose graph edge computation |

### GPU Accuracy Enhancements

- **Gaussian grid smearing** — smooth probability distribution replaces binary occupancy, improving convergence reliability
- **Sub-pixel parabolic interpolation** — refines pose estimate between grid cells for sub-resolution accuracy
- **Inline per-XY reduction** — each GPU thread reduces across all angles internally, eliminating 12.5M-entry buffer writes for loop closure scans
- **Loop closure coarsening** — adaptive spatial thinning for wide-area searches, reducing loop closure latency from 400ms to 25ms

---

## Integration with NVIDIA Isaac ROS

G-SlamBox serves as the primary pose source in a full NVIDIA Isaac ROS autonomy stack:

```
LiDAR (YDLidar TG50, 12Hz)
    │
    ▼
G-SlamBox (GPU SLAM → accurate 2D pose)
    │
    ▼
EKF Fusion (wheel odometry + IMU + SLAM pose)
    │
    ▼
nvblox (real-time 3D reconstruction via RealSense D455)
    │
    ▼
Nav2 (autonomous navigation)
```

G-SlamBox replaces NVIDIA's cuVSLAM in this pipeline, providing 318× more accurate localisation on the same embedded hardware.

---

## Supported Platforms

| Platform | GPU | CUDA Arch | Performance |
|---|---|---|---|
| **Jetson Orin Nano Super** | Ampere (sm_87) | CUDA 12.6 | ~10.65 ms/scan, ~94 Hz |
| **ROG Laptop RTX 5070 Ti** | Blackwell (sm_120) | CUDA 12.8 | ~1.78 ms/scan, ~562 Hz |

Both platforms: 100% convergence, all benchmark tests passing.

---

## Getting Started

> **Note:** Source code will be released after publication. The instructions below describe the intended build process.

### Prerequisites

- ROS 2 Humble or Jazzy
- CUDA 12.6+
- Ceres Solver
- slam_toolbox dependencies

### Build

```bash
# Clone into ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/[username]/g_slambox.git

# Build (Jetson Orin Nano)
cd ~/ros2_ws
colcon build --packages-select gslambox \
  --cmake-args -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc

# Build (Desktop GPU, e.g. RTX 5070 Ti)
colcon build --packages-select gslambox \
  --cmake-args -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc
```

### Run

```bash
source ~/ros2_ws/install/setup.bash

# With the full robot stack
ros2 launch trackedbot_bringup real_robot_Nv_Gsl.launch.py

# Standalone with a bag file
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file config/gslambox_enhanced.yaml \
  -p use_sim_time:=true
```

### Configuration

Key parameters in `gslambox_enhanced.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    correlation_search_space_dimension: 0.5    # Search window (metres)
    correlation_search_space_resolution: 0.01  # Step size (metres) → 51 positions
    coarse_search_angle_offset: 0.5236         # ±30° angular range (radians)
    coarse_angle_resolution: 0.00873           # 0.5° step → 121 angles
    # Total: 51 × 51 × 121 = 314,721 candidate poses per scan
```

---

## GPU Memory Footprint

Total GPU allocation: ~195 MB

| Buffer | Size | Purpose |
|---|---|---|
| Correlation grid | 4 MB | Smeared occupancy grid |
| Response/pose arrays | 53 MB | Scan match results |
| Occupancy grid | 144 MB | Map representation |
| Pose graph buffers | ~8 MB | Graph optimisation data |

Jetson Orin Nano Super uses ~2.2 GB total GPU memory (shared with CPU), leaving headroom for concurrent nvblox 3D mapping.

---

## Citation

Paper in preparation. If you use G-SlamBox in your research, please check back for the citation.

```bibtex
@article{gslambox2026,
  title={G-SlamBox: GPU-Accelerated Correlative Scan Matching for 
         Real-Time 2D SLAM on Embedded Edge Devices},
  author={[Author]},
  journal={[Under Review]},
  year={2026}
}
```

### Intel Research Lab Dataset

```
D. Hähnel, "Intel Research Lab dataset," Radish: The Robotics Data Set 
Repository, University of Freiburg. 
Available: http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
```

---

## Acknowledgements

G-SlamBox is built as a fork of [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) by Steve Macenski. The original CPU correlative scan matcher and karto SLAM framework provided the foundation that G-SlamBox accelerates on GPU.

---

<div align="center">

**G-SlamBox** — Making embedded SLAM faster without making it less accurate.

</div>
