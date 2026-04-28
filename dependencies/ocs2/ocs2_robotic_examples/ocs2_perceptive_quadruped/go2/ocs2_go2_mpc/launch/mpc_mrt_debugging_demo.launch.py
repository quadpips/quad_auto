import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_name = get_package_share_directory("go2_description") + "/urdf/go2_simplified.urdf"

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "description_name",
            default_value=description_name
        ),
        launch.actions.DeclareLaunchArgument(
            "urdfFile",
            default_value=os.path.join(
                get_package_share_directory("go2_description"),
                "urdf/go2_simplified.urdf"
            ),
            description="Absolute path to robot urdf file",
        ),
        launch.actions.DeclareLaunchArgument(
            "taskFile",
            default_value=os.path.join(
                get_package_share_directory("go2_interface"),
                "config/task.info"
            ),
            description="Absolute path to task file",
        ),
        launch.actions.DeclareLaunchArgument(
            "frameFile",
            default_value=os.path.join(
                get_package_share_directory("go2_interface"),
                "config/frame_declaration.info"
            ),
            description="Absolute path to frame file",
        ),
        launch.actions.DeclareLaunchArgument(
            "sqpFile",
            default_value=os.path.join(
                get_package_share_directory("go2_interface"),
                "config/sqp.info"
            ),
            description="Absolute path to SQP file",
        ),    
        launch.actions.DeclareLaunchArgument(
            "gaitCommandFile",
            default_value=os.path.join(
                get_package_share_directory("go2_interface"),
                "config/gait_go2.info"
            ),
            description="Absolute path to OCS2 gait command file",
        ),               
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_custom_quadruped_interface'), 'launch/visualization.launch.py')
            )
            # launch_arguments={
            #     'description_name': LaunchConfiguration('description_name'),
            # }.items()
        ),        
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_odom_tf2",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "world",
                "odom"
            ],
            parameters=[
                {
                    "use_sim_time": True,
                }
            ]
        ),   
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration("description_name")],
        ),       
        launch_ros.actions.Node(
            package='ocs2_go2_commands',
            executable='gait_command_node',
            name='gait_command_node',
            prefix="gnome-terminal --",
            output='screen',
            parameters=[
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile'),
                }
            ]
        ),         
        launch_ros.actions.Node(
            package='ocs2_go2_mpc',
            executable='ocs2_go2_mpc_mpc_mrt_node',
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                    'frameFile': launch.substitutions.LaunchConfiguration('frameFile'),
                    'sqpFile': launch.substitutions.LaunchConfiguration('sqpFile'),
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                }
            ]
        )
    ])


    
    return ld



if __name__ == '__main__':
    generate_launch_description()