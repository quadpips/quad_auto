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

    ####################
    # Launch Arguments #
    ####################
    rviz = True

    #######################
    # Package Directories #
    #######################

    superpixels_path = get_package_share_directory("superpixels")
    config_path = os.path.join(superpixels_path, "cfg", "depth_offline.yaml")

    ############################
    # Declare Launch Arguments #
    ############################
    
    #################
    # Include Nodes #
    #################
    superpixels_node = Node(
        package="superpixels",
        executable="superpixel_depth_segmentation_node",
        name="superpixel_depth_segmentation_node",
        output="screen",
        parameters=[config_path]
    )

    rqt_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
                    superpixels_path, "rviz", "superpixels_depth.rviz",
                )
        ]
    )

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            superpixels_node,
            rviz_node,
            # rqt_node
        ]
    )
