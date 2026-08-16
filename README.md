# CARLA LiDAR Path Tracking & Visualization

This repository contains a Python-based autonomous vehicle control stack for the CARLA Simulator. It demonstrates how to track a predefined set of waypoints using a PD (Proportional-Derivative) controller, dynamically adjust speed based on path curvature, and visualize the vehicle's position in real-time against a LiDAR-generated Point Cloud Map (.pcd) using Open3D.

---

## Features

* **Autonomous Path Tracking:** Uses a dynamic lookahead distance and a PD controller to minimize lateral tracking error.
* **Dynamic Speed Control:** Calculates discrete Menger curvature on the fly to slow the vehicle down during sharp turns and accelerate on straightaways.
* **Real-time 3D Visualization:** Leverages Open3D's non-blocking visualizer to render a LiDAR `.pcd` map, the target trajectory, and a live vehicle marker concurrently with the CARLA simulation.
* **Automatic Spectator Camera:** Binds the CARLA server camera to follow behind and above the ego vehicle for easy monitoring.

---

## Prerequisites

Ensure you have the following installed before running the script:

* Python 3.7 or newer
* [CARLA Simulator](https://carla.org/) (Version 0.9.10 or newer recommended)
* `carla` Python API 
* `open3d` for point cloud visualization
* `numpy` for array operations

Install the required Python packages using pip:

```bash
pip install open3d numpy carla
