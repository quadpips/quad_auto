# OCS2 (ROS2) Toolbox

## Summary
This repository is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), and features that are **NOT** supported at the moment:

* ocs2_mpcnet
* ocs2_raisim
* ocs2_doc

## Installation
### Prerequisites
The OCS2 library is written in C++17. It is tested under Ubuntu 22.04 with library versions as provided in the package sources.

### Dependencies
* C++ compiler with C++17 support
* Ubuntu 22.04
* ROS2 Humble Hawksbill
* Eigen (v3.4)
* Boost C++ (v1.74)

First, cd back to the src/ folder in your ROS2 workspace
```
cd ~/ros2_ws/src
```
Clone HPP-FCL (Branch: devel, [link](https://github.gatech.edu/GeorgiaTechLIDARGroup/hpp-fcl/tree/devel))
```
git clone --recurse-submodules --branch devel https://github.gatech.edu/GeorgiaTechLIDARGroup/hpp-fcl.git
```

Clone Pinocchio (Branch: ros2, [link](https://github.gatech.edu/GeorgiaTechLIDARGroup/pinocchio/tree/ros2))
```
git clone --recurse-submodules --branch ros2 https://github.gatech.edu/GeorgiaTechLIDARGroup/pinocchio.git
```

Clone OCS2 robotic assets (Branch: ros2, [link](https://github.gatech.edu/GeorgiaTechLIDARGroup/ocs2_robotic_assets/tree/ros2)) repository for various robotic assets used in OCS2 unit tests and the robotic examples
```
git clone --branch ros2 https://github.gatech.edu/GeorgiaTechLIDARGroup/ocs2_robotic_assets.git
```

Clone plane segmentation (Branch: main, [link](https://github.gatech.edu/GeorgiaTechLIDARGroup/plane_segmentation/tree/main))
```
git clone --branch main https://github.gatech.edu/GeorgiaTechLIDARGroup/plane_segmentation.git
```

Clone grid_map:
```
git clone --branch humble https://github.com/ANYbotics/grid_map.git
```

Install rest of dependencies from apt
```
sudo apt-get install libmpfr-dev libpcap-dev libglpk-dev
```
Lastly, if you haven't already, clone OCS2 (Branch: ros2_humble, [link](https://github.gatech.edu/GeorgiaTechLIDARGroup/ocs2/tree/ros2_humble))
```
git clone --branch ros2_humble https://github.gatech.edu/GeorgiaTechLIDARGroup/ocs2.git
```

### Etc.
sudo apt install libglpk-dev

sudo apt-get install liburdfdom-dev liboctomap-dev libassimp-dev

sudo apt-get install ros-humble-grid-map