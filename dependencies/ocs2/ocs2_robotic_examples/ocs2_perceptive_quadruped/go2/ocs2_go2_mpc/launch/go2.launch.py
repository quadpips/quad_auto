import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def load_file_content(file_path):  
    with open(file_path, 'r') as file:  
        return file.read()  

def generate_launch_description():
    description_name = get_package_share_directory("go2_description") + "/urdf/go2_payload.urdf"
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_go2_mpc'), 'launch/mpc.launch.py')
            ),
            launch_arguments={
                'robot_name': 'go2',
                'config_name': 'go2_series',
                'description_name': description_name
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
