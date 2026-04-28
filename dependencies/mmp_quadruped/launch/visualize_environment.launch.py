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

      ##########
      envFile = LaunchConfiguration("envFile")

      declare_env_file = DeclareLaunchArgument("envFile",
            default_value=os.path.join(
                  get_package_share_directory("mmp_quadruped"),
                  "config",
                  "environment",
                  "empty.json"
            ),
            description="Path to the environment configuration file."
      )

      ##########
      ## Node ##
      ##########
      env_vis_node = Node(
            package="mmp_quadruped",
            executable="polygon_publisher",
            name="polygon_publisher",
            output="screen",
            parameters=[
                  {
                        "envFile" : LaunchConfiguration("envFile"),
                  }
            ]
      )

      ###########################
      # Full Launch Description #
      ###########################
      return LaunchDescription(
            [
                  declare_env_file,
                  env_vis_node
            ]
      )