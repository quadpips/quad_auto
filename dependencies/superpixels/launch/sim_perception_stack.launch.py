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
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )    

    # grid_map_demos_dir = get_package_share_directory('grid_map_demos')

    # visualization_config_file = LaunchConfiguration("visualization_config")

    # declare_visualization_config_file_cmd = DeclareLaunchArgument(
    #     'visualization_config',
    #     default_value=os.path.join(
    #         grid_map_demos_dir, 'config', 'simple_demo.yaml'),
    #     description='Full path to the Gridmap visualization config file to use')


    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)


    #######################
    # Package Directories #
    #######################

    egocylindrical_path = get_package_share_directory("egocylindrical")
    depth_img_normal_estimation_path = get_package_share_directory("depth_img_normal_estimation")
    superpixels_path = get_package_share_directory("superpixels")
    config_path = os.path.join(superpixels_path, "cfg", "depth_online.yaml")

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

    # grid_map_visualization_node = Node(
    #     package='grid_map_visualization',
    #     executable='grid_map_visualization',
    #     name='grid_map_visualization',
    #     output='screen',
    #     parameters=[visualization_config_file]
    # )

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            set_use_sim_time,
            declare_use_sim_time,
            # declare_visualization_config_file_cmd,
            normal_estimation_ld,
            semantic_egocan_ld,
            superpixels_node,
            # grid_map_visualization_node,
        ]
    )
