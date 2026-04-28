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

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            set_use_sim_time,
            egocylindrical_propagator_node,
        ]
    )
