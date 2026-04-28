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
    #######################
    # Package Directories #
    #######################

    go2_description_path = get_package_share_directory("go2_description")
    go2_gazebo_path = get_package_share_directory("go2_gazebo")

    ####################
    # Launch Arguments #
    ####################
    rviz = "True"
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )    

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    ###############################
    # Include Launch Descriptions #
    ###############################

    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                go2_description_path, "launch", "description_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),        
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                go2_gazebo_path, "launch", "gazebo_mmp.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            set_use_sim_time,
            declare_use_sim_time,
            gazebo_ld,
            description_ld,
        ]
    )
