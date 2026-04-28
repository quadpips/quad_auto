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

    go2_description_path = get_package_share_directory("go2_description")
    go2_interface_path = get_package_share_directory("go2_interface")

    urdfFile = os.path.join(go2_description_path, "urdf/go2_payload.urdf")
    taskFile = os.path.join(go2_interface_path, "config/task.info")
    frameFile = os.path.join(go2_interface_path, "config/frame_declaration.info")
    sqpFile = os.path.join(go2_interface_path, "config/sqp.info")
    controllerConfigFile = os.path.join(go2_interface_path, "config/wbc.info")
    use_force_sensor = False
    contact_threshold = 0
    
    ####################
    # Launch Arguments #
    ####################
    rviz = "True"

    go2_description_path = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")

    go2_xacro_file_path = os.path.join(go2_description_path, "xacro", "robot_payload.xacro")

    # Convert xacro to urdf and publish on /robot_description topic
    robot_description_command = Command(["xacro ", go2_xacro_file_path])

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
        ),
        launch_ros.actions.Node(
            package='go2_interface',
            executable='go2_rosbag_main',
            name='go2_rosbag_main',
            output='screen',
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
            condition=IfCondition(rviz),
            arguments=[
                "-d",
                os.path.join(
                    go2_interface_path, "rviz", "go2_humble_rosbag.rviz",
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
            name="odom_to_odom_cam_offset_tf2",
            output="screen",
            arguments=[
                "0.325",
                "0.005",
                "0.1778",  # 0.2413 m is the height of the T265 camera on the Go2 robot
                "0",
                "0",
                "0",
                "odom",
                "odom_cam_offset"
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        ),  
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()