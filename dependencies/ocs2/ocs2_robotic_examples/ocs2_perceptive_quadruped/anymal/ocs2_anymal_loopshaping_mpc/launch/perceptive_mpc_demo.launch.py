import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + "/config/rviz/demo_config.rviz"
    urdf_model_path = get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value='anymal_c'
        ),
        launch.actions.DeclareLaunchArgument(
            name='config_name',
            default_value='c_series'
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        launch.actions.DeclareLaunchArgument(
            name='perception_parameter_file',
            default_value=get_package_share_directory(
                'convex_plane_decomposition_ros') + '/config/parameters.yaml'
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_model_path],
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            name='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            output='screen',
            parameters=[
                {
                    'config_name': launch.substitutions.LaunchConfiguration('config_name')
                },
                {
                    'forward_velocity': 0.5
                },
                {
                    'terrain_name': 'step.png'
                },
                {
                    'ocs2_anymal_description': urdf_model_path
                },
                {
                    'terrain_scale': 0.35
                },
                {
                    'adaptReferenceToTerrain': True
                },
                {
                    'preprocessing/resolution': 0.04
                },
                {
                    'preprocessing/kernelSize' : 3
                },
                {
                    'preprocessing/numberOfRepeats' : 1
                },
                {
                    'sliding_window_plane_extractor/kernel_size' : 3
                },
                {
                    'sliding_window_plane_extractor/planarity_opening_filter' : 0
                },
                {
                    'sliding_window_plane_extractor/plane_inclination_threshold_degrees' : 30.0
                },
                {
                    'sliding_window_plane_extractor/local_plane_inclination_threshold_degrees' : 35.0
                },
                {
                    'sliding_window_plane_extractor/plane_patch_error_threshold' : 0.02
                },
                {
                    'sliding_window_plane_extractor/min_number_points_per_label' : 4
                },
                {
                    'sliding_window_plane_extractor/connectivity' : 4
                },
                {
                    'sliding_window_plane_extractor/include_ransac_refinement' : True
                },
                {
                    'sliding_window_plane_extractor/global_plane_fit_distance_error_threshold' : 0.025
                },
                {
                    'sliding_window_plane_extractor/global_plane_fit_angle_error_threshold_degrees' : 25.0
                },
                {
                    'ransac_plane_refinement/probability' : 0.001
                },
                {
                    'ransac_plane_refinement/min_points' : 4.0
                },
                {
                    'ransac_plane_refinement/epsilon' : 0.025
                },
                {
                    'ransac_plane_refinement/cluster_epsilon' : 0.041
                },
                {
                    'ransac_plane_refinement/normal_threshold' : 25.0
                },
                {
                    'contour_extraction/marginSize' : 1
                },   
                {
                    'postprocessing/extracted_planes_height_offset' : 0.0
                },
                {
                    'postprocessing/nonplanar_height_offset' : 0.02
                },    
                {
                    'postprocessing/nonplanar_horizontal_offset' : 1
                },
                {
                    'postprocessing/smoothing_dilation_size' : 0.2
                }, 
                {
                    'postprocessing/smoothing_box_kernel_size' : 0.1
                },
                {
                    'postprocessing/smoothing_gauss_kernel_size' : 0.05
                }
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
