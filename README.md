# Occupancy Grid Mapping ROS2 Node

The Occupancy Grid Mapping node is a ROS2 node designed to build occupancy grid maps from raw point cloud data obtained from sensors such as LiDAR. It processes point cloud data and generates a 2D grid map representing the environment's occupancy status.

## Overview

The Occupancy Grid Mapping node subscribes to two synchronized point cloud topics containing raw and obstacle point cloud data. It then processes the data to build an occupancy grid map, where each cell represents the probability of occupancy in the environment.

The node includes functionality for:

- Converting world coordinates to map coordinates.
- Implementing Bresenham's line algorithm for ray tracing.
- Generating an occupancy grid map from point cloud data.
- Publishing the occupancy grid map.

## Prerequisites

- ROS2 Humble or later installed on your system.
- Basic understanding of ROS2 concepts and development.

## Installation

1. Clone this repository into your ROS2 workspace:

    ```bash
    git clone <repository-url>
    ```

2. Build the package using colcon:

    ```bash
    colcon build --symlink-install occupancy_grid_map
    ```

## Usage

1. Launch the Occupancy Grid Mapping node using the provided launch file:

    ```bash
    ros2 launch occupancy_grid_mapping occupancy_grid_mapping.launch.py
    ```

2. Modify the topics to be subscribed for both raw pointcloud and obstacle point cloud in the launch file

    ```bash
        occupancy_grid_map_node = Node(
        package="occupancy_grid_map",
        executable="occupancy_grid_map_exe",
        name="occupancy_grid_map_node",
        namespace="",
        parameters=[config],
        remappings=[
            ("input/pcl_1", "raw_pcl"), #Add your raw pcl topic
            ("input/pcl_2", "obstacle_pcl"), # Add your  obstacle pcl topic
            ("no_ground/objects", "no_ground/pointcloud"),
        ],
        output="both",)
    ```

### Execution time

```bash
Converting to PCL: 2060.82
removeNaNFromPointCloud: 1005.24
creating bins: 6236.94
sorting point within each bin: 1349.55
Initializing free cells: 3665.38
Adding unknown cells: 5485.36
Adding obstacles: 1006.04
full execution time: 31767.3
```

