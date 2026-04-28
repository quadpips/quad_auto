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

    env_prefix = "perceptive/empty"

    go2_description_path = get_package_share_directory("go2_description")
    mmp_quadruped_path = get_package_share_directory("mmp_quadruped")

    ####################
    # Launch Arguments #
    ####################
    hyperparametersFile = os.path.join(mmp_quadruped_path, "config/planning/hyperparameters.info")
    
    mmpSqpFile = os.path.join(mmp_quadruped_path, "config/mmp/sqp.info")
    mmpTaskFile = os.path.join(mmp_quadruped_path, "config/mmp/task.info")
    mmpFrameFile = os.path.join(mmp_quadruped_path, "config/frame_declaration.info")
    mmpUrdfFile = os.path.join(go2_description_path, "urdf/go2_payload.urdf")

    dummyTaskFile = os.path.join(mmp_quadruped_path, "config/mmp/dummy_task.info")
    dummyFrameFile = os.path.join(mmp_quadruped_path, "config/dummy_frame_declaration.info")
    dummyUrdfFile = os.path.join(go2_description_path, "urdf/dummy.urdf")

    envFile = os.path.join(mmp_quadruped_path, "config/environment/" + env_prefix + ".json")
    gaitCommandFile = os.path.join(mmp_quadruped_path, "config/gait.info")

    gait = "standing_trot" # "standing_trot", "standing_walk"

    # Read the URDF file content
    with open(dummyUrdfFile, 'r') as infp:
        robot_desc = infp.read()    

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),    
        launch_ros.actions.Node(
            package="mmp_quadruped",
            executable="go2_mmp",
            name="go2_mmp",
            output="screen",
            prefix="gnome-terminal --",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    'hyperparametersFile': hyperparametersFile,
                    'mmpSqpFile': mmpSqpFile,
                    'mmpTaskFile': mmpTaskFile,
                    'mmpFrameFile': mmpFrameFile,
                    'mmpUrdfFile': mmpUrdfFile,
                    'dummyTaskFile': dummyTaskFile,
                    'dummyFrameFile': dummyFrameFile,
                    'dummyUrdfFile': dummyUrdfFile,
                    'envFile': envFile,
                    'gaitCommandFile': gaitCommandFile,
                    'gait': gait,
                }
            ]
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="dummy_robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc},
                {"publish_frequency": 200.0},
                {"ignore_timestamp": True},
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("/robot_description", "/dummy_description"),
            ]
        ),
        # launch_ros.actions.Node(
        #     package="mmp_quadruped",
        #     executable="terrain_publisher",
        #     name="terrain_publisher",
        #     output="screen",
        #     parameters=[
        #           {
        #                 "use_sim_time": LaunchConfiguration("use_sim_time"),
        #                 "envFile" : envFile
        #           }
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package="mmp_quadruped",
        #     executable="polygon_publisher",
        #     name="polygon_publisher",
        #     output="screen",
        #     parameters=[
        #           {
        #                 "use_sim_time": LaunchConfiguration("use_sim_time"),
        #                 "envFile" : envFile,
        #           }
        #     ]
        # )
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()