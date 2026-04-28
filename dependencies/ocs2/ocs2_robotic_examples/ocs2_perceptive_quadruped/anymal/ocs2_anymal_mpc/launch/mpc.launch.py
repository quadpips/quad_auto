import os
import sys

import launch
from launch.conditions import IfCondition
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_target_pose',
            default_value='false',
            description='Use target pose command',
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_target_velocity',
            default_value='true',
            description='Use target velocity command',
        ),        
        launch.actions.DeclareLaunchArgument(
            name='target_command',
            default_value=os.path.join(
                get_package_share_directory("ocs2_anymal_mpc"),
                "config/c_series/targetCommand.info"
            ),
            description="Absolute path to target command file",
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='robot_description',
        ),
        launch.actions.DeclareLaunchArgument(
            "taskFile",
            default_value=os.path.join(
                get_package_share_directory("ocs2_anymal_mpc"),
                "config/c_series/task.info"
            ),
            description="Absolute path to task file",
        ),    
        launch.actions.DeclareLaunchArgument(
            "urdfFile",
            default_value=LaunchConfiguration('description_name'),
            description="Absolute path to robot urdf file",
        ),           
        launch.actions.DeclareLaunchArgument(
            "frameFile",
            default_value=os.path.join(
                get_package_share_directory("ocs2_anymal_mpc"),
                "config/c_series/frame_declaration.info"
            ),
            description="Absolute path to frame file",
        ),        
        launch.actions.DeclareLaunchArgument(
            "sqpFile",
            default_value=os.path.join(
                get_package_share_directory("ocs2_anymal_mpc"),
                "config/c_series/sqp.info"
            ),
            description="Absolute path to SQP file",
        ),       
        launch.actions.DeclareLaunchArgument(
            "gaitCommandFile",
            default_value=os.path.join(
                get_package_share_directory("ocs2_anymal_mpc"),
                "config/c_series/gait.info"
            ),
            description="Absolute path to OCS2 gait command file",
        ),         
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_quadruped_interface'), 'launch/visualization.launch.py')
            ),
            launch_arguments={
                'description_name': LaunchConfiguration('description_name'),
            }.items()
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration("description_name")],
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
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_mpc_node',
            name='ocs2_anymal_mpc_mpc_node',
            prefix="",
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile'),
                    'urdfFile': LaunchConfiguration('urdfFile'),
                    'frameFile': LaunchConfiguration('frameFile'),
                    'sqpFile': LaunchConfiguration('sqpFile'),
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_dummy_mrt_node',
            name='ocs2_anymal_mpc_dummy_mrt_node',
            prefix="gnome-terminal --",
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile'),
                    'urdfFile': LaunchConfiguration('urdfFile'),
                    'frameFile': LaunchConfiguration('frameFile'),
                    'sqpFile': LaunchConfiguration('sqpFile'),
                }
            ]
        ),
        # launch_ros.actions.Node(
        #     package='ocs2_anymal_commands',
        #     executable='gait_command_node',
        #     name='gait_command_node',
        #     prefix="gnome-terminal --",
        #     output='screen',
        #     parameters=[
        #         {
        #             'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile'),
        #         }
        #     ]
        # ),
        launch_ros.actions.Node(
            package='ocs2_anymal_commands',
            executable='motion_command_node',
            name='motion_command_node',
            prefix="gnome-terminal --",
            output='screen',
            parameters=[
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile'),
                }
            ]
        ),        
        # launch_ros.actions.Node(
            # package='ocs2_anymal_commands',
            # executable='target_command_node',
            # name='target_command_node',
            # prefix="gnome-terminal --",
            # arguments=[LaunchConfiguration('target_command')],
            # condition=IfCondition(LaunchConfiguration('use_target_pose')),
            # output='screen'
        # ),
        # launch_ros.actions.Node(
        #     package='ocs2_anymal_commands',
        #     executable='target_vel_command_node',
        #     name='target_vel_command_node',
        #     condition=IfCondition(LaunchConfiguration('use_target_velocity')),
        #     output='log',
        #     parameters=[
        #         {
        #             'taskFile': LaunchConfiguration('taskFile'),
        #         }
        #     ]
        # ),    
        # launch_ros.actions.Node(
        #     package='ocs2_anymal_commands',
        #     executable='cmd_vel_scorer_node',
        #     name='cmd_vel_scorer_node',
        #     output='screen',
        #     prefix="gnome-terminal --",
        # ),     
        # launch_ros.actions.Node(
        #     package='plotjuggler',
        #     executable='plotjuggler',
        #     name='plotjuggler',
        #     output='screen',
        #     prefix="gnome-terminal --",
        # ),                 
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
