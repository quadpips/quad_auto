#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <envname>"
    exit 1
fi

ENV_NAME=$1

ENV_X=0
ENV_Y=0
ENV_Z=0
ENV_QX=0
ENV_QY=0
ENV_QZ=0
ENV_QW=1

echo "Setting up for $ENV_NAME environment"


if [ "$ENV_NAME" == "empty" ]; then
    ENV_X=0.0
    ENV_Y=0.0
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "balance_beam" ]; then
    ENV_X=-3.0
    ENV_Y=0.0
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "offset_beam" ]; then
    ENV_X=-3.0
    ENV_Y=0.75
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "gap_stones" ]; then
    ENV_X=0.0
    ENV_Y=-0.10
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=-0.7071
    ENV_QW=0.7071   # PI / 2
elif [ "$ENV_NAME" == "gap_stones_spaced" ]; then
    ENV_X=0.0
    ENV_Y=-0.10
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=-0.7071
    ENV_QW=0.7071   # PI / 2    
elif [ "$ENV_NAME" == "pegboard" ]; then
    ENV_X=-2.5
    ENV_Y=1.4
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2    
elif [ "$ENV_NAME" == "ramp_10" ]; then
    ENV_X=-3.0
    ENV_Y=0.0
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "ramp_17" ]; then
    ENV_X=-1.4
    ENV_Y=2.0
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "ramp_25" ]; then
    ENV_X=-1.4
    ENV_Y=2.0
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "ramped_balance_beam_easy" ]; then
    ENV_X=0.0
    ENV_Y=0.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "ramped_stepping_stones_easy" ]; then
    ENV_X=-0.5
    ENV_Y=7.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "ramped_balance_beam" ]; then
    ENV_X=0.0
    ENV_Y=0.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "ramped_stepping_stones" ]; then
    ENV_X=-0.5
    ENV_Y=7.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "rubble" ]; then
    ENV_X=-2.5
    ENV_Y=1.4
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2
elif [ "$ENV_NAME" == "side_stones" ]; then
    ENV_X=-2.5
    ENV_Y=0.15
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2    
elif [ "$ENV_NAME" == "sparse_stones" ]; then
    ENV_X=-3.0
    ENV_Y=1.4
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2    
elif [ "$ENV_NAME" == "stairs" ]; then
    ENV_X=-2.5
    ENV_Y=0.0
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2    
elif [ "$ENV_NAME" == "down_ramp" ]; then
    ENV_X=-0.70
    ENV_Y=0.0
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0
    ENV_QW=1   # PI / 2        
else
    echo "Unknown environment name: $ENV_NAME"
    exit 1
fi

## Pause Gazebo physics
echo "Pausing physics ..."
ros2 service call /pause_physics std_srvs/Empty

## Reset model state
echo "Resetting model state ..."
ros2 service call /gazebo/set_entity_state gazebo_msgs/SetEntityState "
            {
                state: 
                {
                    name: go2, 
                    pose: 
                    {
                        position: {x: "${ENV_X}", y: "${ENV_Y}", z: "${ENV_Z}"}, 
                        orientation: {x: "${ENV_QX}", y: "${ENV_QY}", z: "${ENV_QZ}", w: "${ENV_QW}"}
                    }, twist: 
                    {
                        linear: {x: 0.0, y: 0.0, z: 0.0}, 
                        angular: {x: 0.0, y: 0.0, z: 0.0}
                    }, 
                    reference_frame: world
                }
            }"

## Reset model configuration
echo "Resetting model configuration ..."
ros2 service call /gazebo/set_model_configuration gazebo_msgs/SetModelConfiguration "
                model_name: 'go2'
                urdf_param_name: 'robot_description'
                joint_names:
                    - 'FL_calf_joint'
                    - 'RL_calf_joint'
                    - 'FR_calf_joint'
                    - 'RR_calf_joint'
                    - 'FL_hip_joint'
                    - 'RL_hip_joint'
                    - 'FR_hip_joint'
                    - 'RR_hip_joint'
                    - 'FL_thigh_joint'
                    - 'RL_thigh_joint'
                    - 'FR_thigh_joint'
                    - 'RR_thigh_joint'
                joint_positions:
                    - -2.70
                    - -2.70
                    - -2.70
                    - -2.70
                    - 0.125
                    - 0.125
                    - -0.125
                    - -0.125
                    - 1.125
                    - 1.125
                    - 1.125
                    - 1.125
            "

## Set in CheaterMode 1
echo "Setting CheaterMode to 1 ..."
ros2 service call /CheaterMode go2_interface_msgs/srv/CheaterMode "{cmd: 1}"

## Set in ControlMode 1
echo "Setting ControlMode to 1 ..."
ros2 service call /ControlMode go2_interface_msgs/srv/GO2Cmd "{cmd: 1}"

## Unpause Gazebo physics
echo "Unpausing physics ..."
ros2 service call /unpause_physics std_srvs/Empty