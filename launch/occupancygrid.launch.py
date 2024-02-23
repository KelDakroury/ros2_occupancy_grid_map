from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo

from launch_ros.actions import Node
from launch.actions import GroupAction

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("occupancy_grid_map"),
        "config",
        "occupancy_grid_map.param.yaml",
    )
    # Declare any necessary launch arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )
    occupancy_grid_map_node = Node(
        package="occupancy_grid_map",
        executable="occupancy_grid_map_exe",
        name="occupancy_grid_map_node",
        namespace="",
        parameters=[config],
        remappings=[
            ("input/pcl_1", "/velodyne_points"),
            ("input/pcl_2", "/no_ground/velodyne_points_"),
            ("no_ground/objects", "no_ground/pointcloud"),
        ],
        output="both",
    )
    # Create a group action to launch nodes in parallel
    parallel_group = GroupAction([use_sim_time, occupancy_grid_map_node])

    return LaunchDescription([
        parallel_group,
        LogInfo(msg="All nodes launched in parallel!"),
    ])
