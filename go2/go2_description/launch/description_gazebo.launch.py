import os

import launch
import launch_ros
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    perception_mode = "_camera" # Options: ["", "_camera", "_laser", "_lidar"]

    go2_description_path = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")

    go2_xacro_file_path = os.path.join(go2_description_path, "xacro", "robot" + perception_mode + ".xacro")

    # depthimage_to_laserscan_config_path = os.path.join(go2_description_path, "config", "ros_perception", "depthimage_to_laserscan.yaml")

    # Convert xacro to urdf and publish on /robot_description topic
    robot_description_command = Command(["xacro ", go2_xacro_file_path])

    ###########################
    # Full Launch Description #
    ###########################

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value="true", 
            description="Use simulation (Gazebo) clock if true"
        ),
        # DeclareLaunchArgument(
        #     "perception_mode", 
        #     default_value=perception_mode, 
        #     description="Select perception mode: '', '_camera', '_laser', '_lidar'"
        # ),
        # # Convert depth image to laser scan
        # Node(
        #     package="depthimage_to_laserscan",
        #     executable="depthimage_to_laserscan_node",
        #     name="depthimage_to_laserscan",
        #     remappings=[('depth', '/camera/depth/image_raw'),
        #                 ('depth_camera_info', '/camera/depth/camera_info')],
        #     parameters=[depthimage_to_laserscan_config_path],
        #     output="screen",
        #     condition=IfCondition(str(perception_mode == "_camera")),
        # ),
        # # Convert point cloud to laser scan
        # Node(
        #     package="pointcloud_to_laserscan",
        #     executable="pointcloud_to_laserscan_node",
        #     name="pointcloud_to_laserscan",
        #     remappings=[('cloud_in', '/velodyne_points')],
        #     output="screen",
        #     condition=IfCondition(str(perception_mode == "_lidar")),
        # ),        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_command},
                {"publish_frequency": 200.0},
                {"ignore_timestamp": True},
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
            ] # ,
            # remappings=[("/joint_states", "/simulation_go2/joint_states")],
        ),
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()