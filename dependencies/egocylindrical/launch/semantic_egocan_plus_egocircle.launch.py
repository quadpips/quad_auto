import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():
    fixed_frame_id = "odom"
    scan_frame_id = "egocan_stabilized"

    # Change in egocylindrical_image_to_laserscan if running nodes separately
    range_max = 5.0  
    range_min = 0.1
    floor_dist = 0.325 # TODO: dynamically update based on robot height
    overhead_dist = 0.2

    egocylindrical_propagator_config = os.path.join(
        get_package_share_directory('egocylindrical'),
        'cfg',
        'egocylindrical_propagator.yaml')

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    #################
    # Include Nodes #
    #################
    egocylindrical_propagator_node = Node(
        package="egocylindrical",
        executable="egocylindrical_propagator_node",
        name="egocylindrical_propagator_node",
        output="screen",
        parameters=[egocylindrical_propagator_config]
    )

    point_cloud_node = Node(
        package="egocylindrical",
        executable="point_cloud_node",
        name="point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
            ('cylindrical', 'points'),
        ],
        parameters=[
            {
                'use_sim_time': True,  # Use simulation time if available
            }
        ]
    )

    projected_point_cloud_node = Node(
        package="egocylindrical",
        executable="projected_point_cloud_node",
        name="projected_point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
        ],
        parameters=[
            {
                'use_sim_time': True,  # Use simulation time if available
            }
        ]
    )

    floor_image_node = Node(
        package="egocylindrical",
        executable="floor_image_node",
        name="floor_image_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data')
        ],
        parameters=[
            {
            'use_sim_time': True,  # Use simulation time if available
            'use_raw': False,
            'floor_image_topic': 'floor_image',
            'floor_labels_topic': 'floor_labels',
            'floor_labels_colored_topic': 'floor_labels_colored',
            'floor_normals_topic': 'floor_normals',
            'floor_labels_colored_topic': 'floor_labels_colored'
            }
        ]
    )

    range_image_node = Node(
        package="egocylindrical",
        executable="range_image_node",
        name="egocylindrical_to_range_image",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
        ],
        parameters=[
            {
                'use_sim_time': True,  # Use simulation time if available
                'use_raw': False,
                'image_topic': 'image',
                'can_image_topic': 'can_image'
            }
        ]
    )

    egocylindrical_image_to_laserscan_node = Node(
      package="egocylindrical_image_to_laserscan",
      executable="egocylindrical_image_to_laserscan_node",
      name="egocylindrical_image_to_laserscan_node",
      output="screen",
      remappings=[
          ("image_in", "image"),
          ("camera_info", "/camera/depth/camera_info"),
          ('egocylindrical_points', 'data'),
      ],
      parameters=[
            {
            "use_sim_time": True,
            "fixed_frame_id": fixed_frame_id,
            "scan_frame_id": scan_frame_id,
            "range_max": range_max,
            "range_min": range_min,
            "floor_dist": floor_dist,
            "overhead_dist": overhead_dist,
            }
        ]
    )

    egocircle_node = Node(
        package="egocircle",
        executable="egocircle_node",
        name="egocircle_node",
        output="screen",
        prefix="gnome-terminal --",
        parameters=[
        {
            "use_sim_time": True
        }
        ]        
    )

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            set_use_sim_time,
            egocylindrical_propagator_node,
            point_cloud_node,
            projected_point_cloud_node,
            range_image_node,
            floor_image_node,
            egocircle_node,
            egocylindrical_image_to_laserscan_node
        ]
    )
