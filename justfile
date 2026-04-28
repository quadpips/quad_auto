# Show available commands
help:
    #!/usr/bin/env bash
    just -l

# Fresh build
[group("1. fresh")]
fresh-build:
    #!/usr/bin/env bash
    cd ~/quadpips_ws

    # TODO: check for ~/quadpips_ws

    rm -rf build/
    rm -rf install/
    rm -rf log/

    export MAKEFLAGS="-j 4" 
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython_FIND_VIRTUALENV=ONLY -DPython3_FIND_VIRTUALENV=ONLY --executor sequential --packages-up-to go2_description go2_gazebo go2_interface unitree_joint_controller realsense_gazebo_plugin legged_twist_publisher mmp_quadruped