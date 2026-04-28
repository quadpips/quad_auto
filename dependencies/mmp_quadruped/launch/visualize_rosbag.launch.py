import os
import sys

import launch
import launch_ros.actions
from launch.actions import (DeclareLaunchArgument, ExecuteProcess)
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    env_prefix = "perceptive/sparse_stones"

    go2_description_path = get_package_share_directory("go2_description")
    mmp_quadruped_path = get_package_share_directory("mmp_quadruped")
    go2_interface_path = get_package_share_directory("go2_interface")

    ####################
    # Launch Arguments #
    ####################
    go2_description_path = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")

    go2_xacro_file_path = os.path.join(go2_description_path, "xacro", "camera.xacro")

    # Convert xacro to urdf and publish on /robot_description topic
    robot_description_command = Command(["xacro ", go2_xacro_file_path])

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),    
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_command},
                {"publish_frequency": 200.0},
                {"ignore_timestamp": True},
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
            ] # ,
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    go2_interface_path, "rviz", "go2_mmp.rviz",
                )
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        )
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()