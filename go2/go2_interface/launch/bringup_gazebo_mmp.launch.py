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

    go2_description_path = get_package_share_directory("go2_description")
    go2_interface_path = get_package_share_directory("go2_interface")
    mmp_quadruped_path = get_package_share_directory("mmp_quadruped")

    ####################
    # Launch Arguments #
    ####################
    urdfFile = os.path.join(go2_description_path, "urdf/go2_payload.urdf")
    frameFile = os.path.join(go2_interface_path, "config/frame_declaration.info")

    taskFile = os.path.join(mmp_quadruped_path, "config/mpc/task.info")
    sqpFile = os.path.join(mmp_quadruped_path, "config/mpc/sqp.info")
    controllerConfigFile = os.path.join(mmp_quadruped_path, "config/wbc/wbc.info")

    use_force_sensor = False
    contact_threshold = 0
    rviz = "True"

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),        
        launch_ros.actions.Node(
            package='go2_interface',
            executable='go2_gazebo_main',
            name='go2_gazebo_main',
            output='log',
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    'taskFile': taskFile,
                    'frameFile': frameFile,
                    'sqpFile': sqpFile,
                    'controllerConfigFile': controllerConfigFile,
                    'urdfFile': urdfFile,
                    'use_force_sensor': use_force_sensor,
                    'contact_threshold': contact_threshold,
                }
            ]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
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
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_base_link_tf2",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "base",
                "base_link"
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_base_footprint_tf2",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "base",
                "base_footprint"
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        ),
        # launch_ros.actions.Node(
        #     package="go2_interface",
        #     executable="base_aligned_broadcaster",
        #     name="base_aligned_broadcaster",
        #     output="screen",
        #     # prefix="gnome-terminal --",
        #     parameters=[
        #         {
        #             "use_sim_time": LaunchConfiguration("use_sim_time"),
        #         }
        #     ]
        # )
        # launch_ros.actions.Node(
        #     package='ocs2_go2_commands',
        #     executable='cmd_vel_scorer_node',
        #     name='cmd_vel_scorer_node',
        #     output='log',
        #     # prefix="gnome-terminal --",
        # ),     
        # launch_ros.actions.Node(
        #     package='plotjuggler',
        #     executable='plotjuggler',
        #     name='plotjuggler',
        #     output='log',
        #     # prefix="gnome-terminal --",
        # ),                 
        # launch_ros.actions.Node(
        #     package='go2_interface',
        #     executable='base_footprint_publisher',
        #     output='screen',
        # ),        
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()