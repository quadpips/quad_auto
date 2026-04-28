import os
import sys

import launch
import launch_ros.actions
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    go2_interface_path = get_package_share_directory("go2_interface")

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
        ),
        launch_ros.actions.Node(
            package="go2_interface",
            executable="get_low_state_node",
            
        )
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()