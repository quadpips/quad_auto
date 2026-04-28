# Multi-modal planning for Quadrupeds

<p>
    <img align="center" width="1000" src="./assets/diagram2.png" alt="empty"> 
</p>

## Introduction
This is a C++/ROS2 implementation of our graph search and trajectory optimization-based footstep planner.

## Dependencies

### ROS2
This package has been tested on Ubuntu 22.04 / ROS2 Humble. 

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/index.html).

2. Install the following ROS2 Humble packages:
    ```sh
    sudo apt-get install ros-humble-controller-manager ros-humble-control-toolbox ros-humble-realtime-tools ros-humble-joint-state-broadcaster ros-humble-gazebo-ros ros-humble-angles ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-robot-localization ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-gazebo-plugins
    ```

### legged_software 
1. Follow the legged_software installation instructions on the `feat/go2` branch <a href="https://github.com/max-assel/legged_software/tree/feat/go2">here</a>.

### superpixels
1. Follow the superpixels installation instructions on the `ros2_humble` branch <a href="https://github.com/max-assel/superpixels/tree/ros2_humble">here</a>.

## Building

Now you can build the `mmp_quadruped` package. 

```
MAKEFLAGS="-j 4" colcon build --symlink-install --executor sequential --mixin rel-with-deb-info --packages-up-to mmp_quadruped
```

Building ocs2 can be quite computationally expensive, so `MAKEFLAGS="-j 4"` limits the number (4) of cores used during build, and `--executor sequential` makes sure packages are build sequentially, not in parallel. `--mixin rel-with-deb-info` builds the release version of the code, not the debugging version in order to get better performance, though this needs colcon mixin to be set up. Feel free to adjust these to your liking. I would also recommend that you export this alias into your bashrc so that you do not have to copy this in every time

```
echo "alias buildros2='cd ~/ros2_ws && MAKEFLAGS="-j 4" colcon build --symlink-install --executor sequential --mixin rel-with-deb-info --packages-up-to'" >> ~/.bashrc  
```
## Running
First, decide on what environment you want to test in. Set this environment as `env_prefix` in `go2_gazebo/launch/gazebo_mmp.launch.py` AND as `env_prefix` in `mmp_quadruped/launch/online_perceptive_mmp_sim.launch.py`.


1. To run the planner in Gazebo, first launch gazebo and spawn the Go2:
    ```
    ros2 launch go2_interface run_gazebo_mmp.launch.py
    ```

2. To bring up our autonomy codebase, run:
    ```
    ros2 launch go2_interface bringup_gazebo_mmp.launch.py
    ```

3. Initialize the testing setup by running
    ```
    ./init_test.sh ENV_NAME
    ```
    Where `ENV_NAME` should be the same environment name that is set as `env_prefix`.

4. Launch the superpixels based perception pipeline:
    ```
    ros2 launch superpixels sim_perception_stack.launch.py
    ```

5. Finally, launch the planner:
    ```
    ros2 launch mmp_quadruped online_perceptive_mmp_sim.launch.py
    ```

6. To begin planning, set the framework into Control Mode 4:
    ```
    ros2 service call /ControlMode go2_interface_msgs/srv/GO2Cmd "{cmd: 4}"
    ```

## Citation
If would like to cite this work, please use the following format:
```
@INPROCEEDINGS{10610248,
  author={Asselmeier, Max and Ivanova, Jane and Zhou, Ziyi and Vela, Patricio A. and Zhao, Ye},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Hierarchical Experience-informed Navigation for Multi-modal Quadrupedal Rebar Grid Traversal}, 
  year={2024},
  volume={},
  number={},
  pages={8065-8072},
  keywords={Torso;Costs;Navigation;Statistical analysis;Hardware;Planning;Quadrupedal robots},
  doi={10.1109/ICRA57147.2024.10610248}
}
```