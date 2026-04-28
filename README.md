# QuadAuto (Quadrupedal Autonomy)

<p align="center">
    <a href="https://arxiv.org/abs/2501.00112">arXiv</a> •
    <a href="https://google.com">Website</a> •
    <a href="##Citation">Citation</a>
</p>

<p>
    <img align="center" width="450" src="./assets/stairs.gif" alt="empty"> 
    <img align="center" width="550" src="./assets/stones.gif" alt="empty"> 
</p>

<p>
    <img align="center" width="1000" src="./assets/nav.gif" alt="empty"> 
</p>

## Introduction

This repository is built upon the [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software), but allows more convenient integration with other ROS2 packages such as OCS2 or other perception modules. Right now, this repository only supports the Unitree Go2 quadruped platform.

## Dependencies

### Git Dependencies

This repository uses Git Large File Storage to maintain robot meshes. 

1. Follow the directions [here](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage) to install Git LFS.

### ROS Dependencies
This framework runs on ROS2 Humble. We provide installation through [pixi](https://pixi.prefix.dev/latest/), so you should be able to run this on any OS.

## Installation

1. Install proxsuite as a QP solver for WBC:
    ```bash
    cd ~/
    git clone --recurse-submodules https://github.gatech.edu/GeorgiaTechLIDARGroup/proxsuite.git
    cd proxsuite
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF
    make
    sudo make install
    ```

2. Now you can build the `go2_interface` package. First, set up the pixi virtual environment.
    ```bash
    # Install pixi
    curl -fsSL https://pixi.sh/install.sh | sh # You might have to source your config again

    # Install QuadPiPS
    mkdir -p ~/quadpips_ws/src && cd ~/quadpips/src
    git clone https://github.gatech.edu/GeorgiaTechLIDARGroup/legged_software.git --branch pixi --recursive && cd legged_autonomy
    git lfs pull
    pixi install && pixi shell
    just fresh-build
    ```

3. We follow the communication protocol of [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) which uses cyclonedds.

    First, ensure that ROS2 uses cyclonedds for its RMW configuration:
    ```bash
    echo export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp >> ~/.bashrc
    echo export CYCLONEDDS_URI=~/quadpips_ws/src/legged_software/cyclonedds.xml >> ~/.bashrc
    source ~/.bashrc
    ```

    And update the network name `enp128s31f6` in the case yours is something different.

## Demonstrations

### Teleop
To run gazebo simulation:

```bash
ros2 launch go2_interface run_gazebo.launch.py
```

To bring up the robot, including state estimators, controllers, and visualizers:

```bash
ros2 launch go2_interface bringup_gazebo.launch.py
```

Note that it will take some time to compile the CppAD files depending on the capabilities of your PC. The parameter `recompileLibraries` is set to `true` by default. This means that every time you run this script, the CppAd files will re-compile. Once you have everything building and running properly, you can disable this, but I would suggest you keep it on until everything is running correctly to ensure that you are running with the most up-to-date libraries.

You can either use the Invariant Extended Kalman filter from our codebase for base odometry estimation, or enable "Cheater Mode" which uses ground truth odometry estimation from the simulation API:

```bash
ros2 service call /CheaterMode go2_interface_msgs/srv/CheaterMode "{cmd: 1}"
```

Once the CppAd files are done compiling, the robot will then enter the passive mode. You need to manually send ROS services to change the mode. First, stand the robot up

```bash
ros2 service call /ControlMode go2_interface_msgs/srv/GO2Cmd "{cmd: 1}" // 0 passive; 1 standup; 4 perception nonlinear MPC from OCS2
```

And once the robot has stood up, you can begin running the MPC

```bash
ros2 service call /ControlMode go2_interface_msgs/srv/GO2Cmd "{cmd: 4}" // 0 passive; 1 standup; 4 perception nonlinear MPC from OCS2
```

Quick note: the `ControlMode 4` uses a nonlinear MPC based on OCS2. This requires a nontrivial amount of computational power, but it leverages the full-body kinematics plus centroidal dynamics. 

You should see a pop-up terminal that states

```bash
Enter the desired gait, for the list of available gait enter "list":
```

Define which gait you want to use. A few good options are `trot`, `standing_trot`, and `standing_walk`.

```bash
trot # options: trot, standing_trot, standing_walk.
```

Once you have input your gait, you can run our keyboard teleop node:

```bash
ros2 run legged_twist_publisher legged_twist_publisher
```

Then, you can use WASD to translate, Q/E to rotate, and X to send a zero velocity commands. Z will exit the node, so only press this when you are done.

TODOs
- [ ] Install from scratch
- [ ] Sanity check inekf kinematics
- [ ] Build proxsuite inside pixi
- [ ] Add ANYmal
- [ ] Single package for all quadrupeds
- [ ] MuJoCo sim
- [ ] Add Nav2 demo
- [ ] Add OCS2 for G1

## Acknowledgements

## Citation
