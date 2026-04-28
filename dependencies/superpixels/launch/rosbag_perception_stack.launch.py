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
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )    

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=False)


    #######################
    # Package Directories #
    #######################

    anymal_interface_path = get_package_share_directory("anymal_interface")
    egocylindrical_path = get_package_share_directory("egocylindrical")
    depth_img_normal_estimation_path = get_package_share_directory("depth_img_normal_estimation")
    superpixels_path = get_package_share_directory("superpixels")
    config_path = os.path.join(superpixels_path, "cfg", "depth_rosbag.yaml")

    ############################
    # Declare Launch Arguments #
    ############################
    
    #################
    # Include Nodes #
    #################
    normal_estimation_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                depth_img_normal_estimation_path, "launch", "normal_estimation_sim.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    semantic_egocan_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                egocylindrical_path, "launch", "semantic_egocan.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )    

    superpixels_node = Node(
        package="superpixels",
        executable="superpixel_depth_segmentation_node",
        name="superpixel_depth_segmentation_node",
        output="screen",
        parameters=[config_path]
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                anymal_interface_path, "rviz", "anymal_mmp.rviz",
            )
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ]
    )


    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            set_use_sim_time,
            declare_use_sim_time,
            normal_estimation_ld,
            semantic_egocan_ld,
            superpixels_node,
            rviz_node,
        ]
    )
