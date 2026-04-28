
import argparse
import os
import yaml
import numpy as np

from utils import rotation_matrix
from color import get_constant_color

def make_stairs(args):
    print("[make_stairs()]")

    env_yaml = {}

    stones = [] # 

    num_h_stones = int(args.num_h_stones)
    num_v_stones = int(args.num_v_stones)
    stone_length = float(args.stone_length)
    stone_width = float(args.stone_width)
    stone_height = float(args.stone_height)
    x_spacing = float(args.x_spacing)
    y_spacing = float(args.y_spacing)
    z_spacing = float(args.z_spacing)

    env_frame_x = float(args.env_frame_x)
    env_frame_y = float(args.env_frame_y)
    env_frame_z = float(args.env_frame_z)
    env_frame_roll = float(args.env_frame_roll)
    env_frame_pitch = float(args.env_frame_pitch)
    env_frame_yaw = float(args.env_frame_yaw)

    stone_scale = [stone_length, stone_width, stone_height]

    env_frame_pose = np.array([[env_frame_x],
                                [env_frame_y], 
                                [env_frame_z], 
                                [env_frame_roll], 
                                [env_frame_pitch], 
                                [env_frame_yaw]]) # TODO: make sure angles are in correct order

    rot_mat = rotation_matrix(env_frame_roll, env_frame_pitch, env_frame_yaw)

    stone_center_pose_env_frame = np.array([[0.0], 
                                            [0.0],
                                            [0.0]])

    ##########################################################################
    ############################ Main environment ############################
    ##########################################################################

    stair_length = 0.20
    stair_width = 3.0
    stair_height = 0.05

    ## flat
    stone_center_pose_env_frame = np.array([[0.0], # 
                                            [0.0],
                                            [(stair_height / 2.0)]])

    rot_mat = rotation_matrix(env_frame_roll, env_frame_pitch, env_frame_yaw)

    num_stairs = num_v_stones

    counter = 0
    for y_stone_i in range(0, num_stairs):
        # counter += 1
        # for x_stone_i in range(0, num_v_stones):

        counter += 1

        length_factor = 1.0 # 0.5 + np.random.random_sample() # 1.0 # 
        width_factor = 1.0 # 0.5 + np.random.random_sample() # 1.0 # 

        stone_scale = [stair_length * length_factor, 
                        stair_width * width_factor, 
                        stair_height]

        color = get_constant_color()

        # sample = np.random.randint(5)
        # if (sample == 6 or sample == 3):  # (raised)
        # if (counter % 2 == 0): # start_or_goal_stone
        stone_pose_offset_world_frame = np.matmul(rot_mat, stone_center_pose_env_frame)
        # print('stone_pose_offset_world_frame: ', stone_pose_offset_world_frame)

        x_noise_lims = [-0.03, 0.03] # m
        x_noise = 0.0 # x_noise_lims[0] + np.random.random_sample() * (x_noise_lims[1] - x_noise_lims[0]) # 0.0 # 
        
        y_noise_lims = [-0.03, 0.03] # m
        y_noise = 0.0 # y_noise_lims[0] + np.random.random_sample() * (y_noise_lims[1] - y_noise_lims[0]) # 0.0 # 
        
        z_noise_lims = [0.0, 0.075] # m
        z_noise = 0.0 # z_noise_lims[0] + np.random.random_sample() * (z_noise_lims[1] - z_noise_lims[0]) # 0.0 # 

        roll_noise_lims = [-0.25, 0.25] # radians
        roll_noise = 0.0 # roll_noise_lims[0] + np.random.random_sample() * (roll_noise_lims[1] - roll_noise_lims[0]) # 0.0 # 
        
        pitch_noise_lims = [-0.25, 0.25] # radians
        pitch_noise = 0.0 # pitch_noise_lims[0] + np.random.random_sample() * (pitch_noise_lims[1] - pitch_noise_lims[0]) # 0.0 # 

        yaw_noise_lims = [-0.25, 0.25] # radians
        yaw_noise = 0.0 # yaw_noise_lims[0] + np.random.random_sample() * (yaw_noise_lims[1] - yaw_noise_lims[0]) # 0.0 # 

        stone_pose_offset_world_frame_extended = np.array([[stone_pose_offset_world_frame[0, 0] + x_noise], 
                                                            [stone_pose_offset_world_frame[1, 0] + y_noise],
                                                            [stone_pose_offset_world_frame[2, 0] + z_noise], 
                                                            [roll_noise], 
                                                            [pitch_noise], 
                                                            [yaw_noise]])
        
        stone_pose = np.add(env_frame_pose, stone_pose_offset_world_frame_extended)
        
        # print('stone_pose: ', stone_pose)
        # print('float(stone_pose[1]): ', float(stone_pose[1]))
        # "pose" : stone_pose,
        stone_yaml = {"name" : "h" + str(y_stone_i) + "v" + str(0), 
                    "pose" : [float(stone_pose[0]),
                                float(stone_pose[1]),
                                float(stone_pose[2]),
                                float(stone_pose[3]),
                                float(stone_pose[4]),
                                float(stone_pose[5])],
                    "scale" : [stone_scale[0], stone_scale[1], stone_scale[2]],
                    "color" : color } 
        stones.append(stone_yaml)

        # Increment x position and z position
        stone_center_pose_env_frame[0, 0] += stair_length
        stone_center_pose_env_frame[2, 0] += stair_height

    #########################################################################
    ############################ Ending platform ############################
    #########################################################################

    platform_scale = [stair_width, stair_width, stair_height]

    # half length for platform itself and half length at angle for ramp
    platform_length_offset = stair_length * (num_stairs - 0.5) + (stair_width / 2.0)

    platform_height_offset = stair_height * num_stairs

    platform_center_pose_env_frame = np.array([[platform_length_offset], 
                                                [0.0],
                                                [platform_height_offset]])

    color = get_constant_color()

    platform_pose_offset_world_frame = np.matmul(rot_mat, platform_center_pose_env_frame)
    # print('platform_pose_offset_world_frame: ', platform_pose_offset_world_frame)


    platform_pose_offset_world_frame_extended = np.array([[platform_pose_offset_world_frame[0, 0]], 
                                                            [platform_pose_offset_world_frame[1, 0]],
                                                            [platform_pose_offset_world_frame[2, 0]], 
                                                            [0.0], 
                                                            [0.0], 
                                                            [0.0]])
    
    platform_pose = np.add(env_frame_pose, platform_pose_offset_world_frame_extended)
    
    # print('platform_pose: ', platform_pose)
    # print('float(platform_pose[1]): ', float(platform_pose[1]))
    # "pose" : platform_pose,
    platform_yaml = {"name" : "h" + str(num_stairs) + "v" + str(0), 
                        "pose" : [float(platform_pose[0]),
                                    float(platform_pose[1]),
                                    float(platform_pose[2]),
                                    float(platform_pose[3]),
                                    float(platform_pose[4]),
                                    float(platform_pose[5])],
                        "scale" : [platform_scale[0], platform_scale[1], platform_scale[2]],
                        "color" : color } 
    stones.append(platform_yaml)

    env_yaml["models"] = stones

    filename = args.filename
    filepath = os.path.join("/home/masselmeier3/ros2_ws/src/mmp_quadruped/worlds/perceptive/" + filename + ".yaml")

    with open(filepath, 'w') as yaml_file:
        yaml.dump(env_yaml, yaml_file, default_flow_style=False)

    return
