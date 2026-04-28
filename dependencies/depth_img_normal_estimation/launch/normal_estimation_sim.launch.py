import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),        
        launch_ros.actions.Node(
            package='depth_img_normal_estimation',
            executable='depth_img_normal_estimation_node',
            name='depth_img_normal_estimation_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration("use_sim_time")
                },
                {
                    'camera_depth_topic': '/camera/depth/image_raw'
                },
                {
                    'config_path': get_package_share_directory('depth_img_normal_estimation') + '/cfg/sim.yaml'
                },
                {
                    'hardware': False
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
