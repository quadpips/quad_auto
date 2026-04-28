import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    
    #######################
    # Package Directories #
    #######################

    go2_description_path = get_package_share_directory("go2_description")
    mmp_quadruped_path = get_package_share_directory("mmp_quadruped")
    go2_gazebo_path = get_package_share_directory("go2_gazebo")
    launch_dir = os.path.join(go2_description_path, "launch")
    gazebo_config_path = os.path.join(go2_gazebo_path, "config/gazebo.yaml")

    robot_name = "go2"
    world_init_x = "0.0"
    world_init_y = "0.0"
    world_init_z = "0.5"
    world_init_heading = "0"  # 90 degrees in radians
    headless = "False"
    env_prefix = "perceptive/empty"

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(mmp_quadruped_path, "worlds/" + env_prefix + ".world"), description="Gazebo world name"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),
        ExecuteProcess(
            cmd=[
                "gzserver",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
                LaunchConfiguration("world"),
                '--ros-args',
                '--params-file',
                gazebo_config_path
            ],
            cwd=[launch_dir],
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(PythonExpression([" not ", headless])),
            cmd=["gzclient"],
            cwd=[launch_dir],
            output="screen",
        ),
        launch_ros.actions.Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="urdf_spawner",
            output="screen",
            arguments=[
                "-entity", robot_name,
                "-topic", "/robot_description",
                "-x", world_init_x,
                "-y", world_init_y,
                "-z", world_init_z,
                "-R", "0",
                "-P", "0",
                "-Y", world_init_heading,
                "-unpause"
            ],
        ),
        launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            name="joint_state_broadcaster_spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        ),
        launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            name="unitree_joint_controller_spawner",
            arguments=["unitree_joint_controller"],
            output="screen",
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