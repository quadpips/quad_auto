import argparse
import os
import yaml
import numpy as np

from utils import rotation_matrix
from color import get_constant_color

def make_ramp(args):
    print("[make_ramp()]")

    grid_yaml = {}

    ramps = [] # 

    ramp_length = 3.0 # float(args.ramp_length)
    ramp_width = 3.0 # float(args.ramp_width)
    ramp_height = 0.10 # float(args.ramp_height)

    env_frame_x = float(args.env_frame_x)
    env_frame_y = float(args.env_frame_y)
    env_frame_z = float(args.env_frame_z)
    env_frame_roll = 0.0
    env_frame_pitch = 0.0
    env_frame_yaw = 0.0

    ramp_scale = [ramp_length, ramp_width, ramp_height]

    grid_frame_pose = np.array([[env_frame_x],
                                [env_frame_y], 
                                [env_frame_z], 
                                [env_frame_roll], 
                                [env_frame_pitch], 
                                [env_frame_yaw]]) # TODO: make sure angles are in correct order

    rot_mat = rotation_matrix(env_frame_roll, env_frame_pitch, env_frame_yaw)

    ##########################################################################
    ############################ Main environment ############################
    ##########################################################################

    env_frame_x = float(args.env_frame_x)
    env_frame_y = float(args.env_frame_y)
    env_frame_z = float(args.env_frame_z)
    env_frame_roll = float(args.env_frame_roll)
    env_frame_pitch = float(args.env_frame_pitch)
    env_frame_yaw = float(args.env_frame_yaw)

    grid_frame_pose = np.array([[env_frame_x],
                                [env_frame_y], 
                                [env_frame_z], 
                                [env_frame_roll], 
                                [env_frame_pitch], 
                                [env_frame_yaw]])

    rot_mat = rotation_matrix(env_frame_roll, env_frame_pitch, env_frame_yaw)

    ramp_scale = [ramp_length, 
                    ramp_width, 
                    ramp_height]

    ramp_center_pose_grid_frame = np.array([[0.0], 
                                            [0.0],
                                            [0.0]])

    color = get_constant_color()

    ###########################################################################
    ############################ Starting platform ############################
    ###########################################################################

    platform_scale = ramp_scale

    # half length for platform itself and half length at angle for ramp
    platform_length_offset = 0.0

    platform_height_offset = 0.0

    platform_center_pose_grid_frame = np.array([[0.175], 
                                                [0.0],
                                                [- platform_scale[2] / 2.0]])

    platform_pose_offset_world_frame = np.matmul(rot_mat, platform_center_pose_grid_frame)
    # print('platform_pose_offset_world_frame: ', platform_pose_offset_world_frame)


    platform_pose_offset_world_frame_extended = np.array([[platform_pose_offset_world_frame[0, 0]], 
                                                            [platform_pose_offset_world_frame[1, 0]],
                                                            [platform_pose_offset_world_frame[2, 0]], 
                                                            [0.0], 
                                                            [0.0], 
                                                            [0.0]])
    
    platform_pose = np.add(grid_frame_pose, platform_pose_offset_world_frame_extended)
    
    # print('platform_pose: ', platform_pose)
    # print('float(platform_pose[1]): ', float(platform_pose[1]))
    # "pose" : platform_pose,
    platform_yaml = {"name" : "h" + str(1) + "v" + str(0), 
                        "pose" : [float(platform_pose[0]),
                                    float(platform_pose[1]),
                                    float(platform_pose[2]),
                                    float(platform_pose[3]),
                                    float(platform_pose[4]),
                                    float(platform_pose[5])],
                        "scale" : [platform_scale[0], platform_scale[1], platform_scale[2]],
                        "color" : color } 
    ramps.append(platform_yaml)

    ramp_pitch = 0.175

    ramp_center_pose_grid_frame = np.array([[platform_scale[0]], 
                                            [0.0],
                                            [- platform_scale[2] / 2.0]])

    ramp_pose_offset_world_frame = np.matmul(rot_mat, ramp_center_pose_grid_frame)
    # print('ramp_pose_offset_world_frame: ', ramp_pose_offset_world_frame)

    ramp_height_offset = (ramp_length / 2.0) * np.sin(-ramp_pitch)

    print('ramp_length: ', ramp_length)
    print('ramp_pitch: ', ramp_pitch)
    print('ramp_height_offset: ', ramp_height_offset)

    ramp_pose_offset_world_frame_extended = np.array([[ramp_pose_offset_world_frame[0, 0]], 
                                                        [ramp_pose_offset_world_frame[1, 0]],
                                                        [ramp_height_offset + ramp_pose_offset_world_frame[2, 0]], 
                                                        [0.0], 
                                                        [ramp_pitch], 
                                                        [0.0]])
    
    ramp_pose = np.add(grid_frame_pose, ramp_pose_offset_world_frame_extended)
    
    # print('ramp_pose: ', ramp_pose)
    # print('float(ramp_pose[1]): ', float(ramp_pose[1]))
    # "pose" : ramp_pose,
    ramp_yaml = {"name" : "h" + str(0) + "v" + str(0), 
                "pose" : [float(ramp_pose[0]),
                            float(ramp_pose[1]),
                            float(ramp_pose[2]),
                            float(ramp_pose[3]),
                            float(ramp_pose[4]),
                            float(ramp_pose[5])],
                "scale" : [ramp_scale[0], ramp_scale[1], ramp_scale[2]],
                "color" : color } 
    ramps.append(ramp_yaml)

    #########################################################################
    ############################ Ending platform ############################
    #########################################################################

    platform_scale = ramp_scale

    # half length for platform itself and half length at angle for ramp
    platform_length_offset = platform_scale[0] + (ramp_length / 2.0) + (ramp_length / 2.0) * np.cos(-ramp_pitch)

    platform_height_offset = - platform_scale[2] / 2.0 + (ramp_length) * np.sin(-ramp_pitch)

    platform_center_pose_grid_frame = np.array([[platform_length_offset], 
                                                [0.0],
                                                [platform_height_offset]])

    color = get_constant_color()

    platform_pose_offset_world_frame = np.matmul(rot_mat, platform_center_pose_grid_frame)
    # print('platform_pose_offset_world_frame: ', platform_pose_offset_world_frame)


    platform_pose_offset_world_frame_extended = np.array([[platform_pose_offset_world_frame[0, 0]], 
                                                            [platform_pose_offset_world_frame[1, 0]],
                                                            [platform_pose_offset_world_frame[2, 0]], 
                                                            [0.0], 
                                                            [0.0], 
                                                            [0.0]])
    
    platform_pose = np.add(grid_frame_pose, platform_pose_offset_world_frame_extended)
    
    # print('platform_pose: ', platform_pose)
    # print('float(platform_pose[1]): ', float(platform_pose[1]))
    # "pose" : platform_pose,
    platform_yaml = {"name" : "h" + str(1) + "v" + str(0), 
                        "pose" : [float(platform_pose[0]),
                                    float(platform_pose[1]),
                                    float(platform_pose[2]),
                                    float(platform_pose[3]),
                                    float(platform_pose[4]),
                                    float(platform_pose[5])],
                        "scale" : [platform_scale[0], platform_scale[1], platform_scale[2]],
                        "color" : color } 
    ramps.append(platform_yaml)

    grid_yaml["models"] = ramps

    filename = args.filename
    filepath = os.path.join("/home/masselmeier3/ros2_ws/src/mmp_quadruped/worlds/perceptive/" + filename + ".yaml")

    with open(filepath, 'w') as yaml_file:
        yaml.dump(grid_yaml, yaml_file, default_flow_style=False)