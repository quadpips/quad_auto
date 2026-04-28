# egocylindrical

##  How to use hybrid-frame:
- Set the following private parameters of the `depth_to_egocylindrical_node(let)`:
  - `fixed_frame_id`: the frame that should be considered fixed wrt time (your odometry frame is fine)
  - `orientation_fixed_frame_id`: the egocan's orientation will be relative to this frame
  - `origin_fixed_frame_id`: the egocan's origin will track the origin of this frame. Currently, this must be set to the depth camera's frame
- [Optional] Send `PoseStamped` messages to `[depth_to_egocylindrical_node(let)]/desired_pose` specifying the desired egocan orientation (currently, any position component is ignored)

Note: The previous behavior of locking the egocan frame to the camera frame can be achieved by either not setting values for `orientation_fixed_frame_id` and `origin_fixed_frame_id`, or by setting them both to the depth image frame.

## Running alongside go2

### Building

Need source pcl_ros

### Branches
- `legged_software` : `dev/mmp`

### Installation

First, you must start up the go2 in Gazebo

```
roslaunch go2_interface run_gazebo_egocan.launch
```

Start up autonomy stack (FSM, controllers, visualizers, publishers)

```
rosrun go2_interface go2_gazebo_main
```

Stand up robot

```
rosservice call /ControlMode 1
```

Activate cheater mode

``` 
rosservice call /CheaterMode 1
```

Start up egocan

```
roslaunch egocylindrical egocan.launch
```

Start up move_base

```
roslaunch legged_move_base go2_sim_move_base.launch
```

