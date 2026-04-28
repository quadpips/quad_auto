
import argparse
import os
import yaml
import numpy as np

from utils import rotation_matrix
from color import get_constant_color

def make_side_stones(args):
    print("[make_side_stones()]")

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

    ###########################################################################
    ############################ Starting platform ############################
    ###########################################################################

    plank_width = 2.0 
    plank_length = 0.30
    plank_height = 0.10

    start_platform_width = 50.0
    start_platform_length = 50.0
    start_platform_height = 0.10

    start_platform_scale = [start_platform_width, start_platform_length, start_platform_height]

    stone_center_pose_env_frame = np.array([[-(start_platform_width / 2.0) - (plank_width / 2.0)], #
                                            [0.0],
                                            [-(start_platform_height / 2.0)]])

    stone_pose_offset_world_frame = np.matmul(rot_mat, stone_center_pose_env_frame)
    # print('stone_pose_offset_world_frame: ', stone_pose_offset_world_frame)

    color = get_constant_color()

    stone_pose_offset_world_frame_extended = np.array([[stone_pose_offset_world_frame[0, 0]], 
                                                        [stone_pose_offset_world_frame[1, 0]],
                                                        [stone_pose_offset_world_frame[2, 0]], 
                                                        [0.0], 
                                                        [0.0], 
                                                        [0.0]])
    
    stone_pose = np.add(env_frame_pose, stone_pose_offset_world_frame_extended)
    
    # print('stone_pose: ', stone_pose)
    # print('float(stone_pose[1]): ', float(stone_pose[1]))
    # "pose" : stone_pose,
    stone_yaml = {"name" : "h" + str(0) + "v" + str(0), 
                    "pose" : [float(stone_pose[0]),
                                float(stone_pose[1]),
                                float(stone_pose[2]),
                                float(stone_pose[3]),
                                float(stone_pose[4]),
                                float(stone_pose[5])],
                    "scale" : [start_platform_scale[0], start_platform_scale[1], start_platform_scale[2]],
                    "color" : color } 
    stones.append(stone_yaml)

    ########################################################################
    ############################ Plank platform ############################
    ########################################################################

    plank_width = 2.0 
    plank_length = 0.20
    plank_height = 0.10

    plank_platform_scale = [plank_width, plank_length, plank_height]

    stone_center_pose_env_frame = np.array([[0.0], #
                                            [0.65],
                                            [-(plank_height / 2.0)]])
    stone_pose_offset_world_frame = np.matmul(rot_mat, stone_center_pose_env_frame)
    # print('stone_pose_offset_world_frame: ', stone_pose_offset_world_frame)

    color = get_constant_color()

    stone_pose_offset_world_frame_extended = np.array([[stone_pose_offset_world_frame[0, 0]], 
                                                        [stone_pose_offset_world_frame[1, 0]],
                                                        [stone_pose_offset_world_frame[2, 0]], 
                                                        [0.0], 
                                                        [0.0], 
                                                        [0.0]])
    
    stone_pose = np.add(env_frame_pose, stone_pose_offset_world_frame_extended)
    
    # print('stone_pose: ', stone_pose)
    # print('float(stone_pose[1]): ', float(stone_pose[1]))
    # "pose" : stone_pose,
    stone_yaml = {"name" : "h" + str(0) + "v" + str(0), 
                    "pose" : [float(stone_pose[0]),
                                float(stone_pose[1]),
                                float(stone_pose[2]),
                                float(stone_pose[3]),
                                float(stone_pose[4]),
                                float(stone_pose[5])],
                    "scale" : [plank_platform_scale[0], plank_platform_scale[1], plank_platform_scale[2]],
                    "color" : color } 
    stones.append(stone_yaml)    

    stone_center_pose_env_frame = np.array([[0.0], #
                                            [-0.65],
                                            [-(plank_height / 2.0)]])

    stone_pose_offset_world_frame = np.matmul(rot_mat, stone_center_pose_env_frame)
    # print('stone_pose_offset_world_frame: ', stone_pose_offset_world_frame)

    color = get_constant_color()

    stone_pose_offset_world_frame_extended = np.array([[stone_pose_offset_world_frame[0, 0]], 
                                                        [stone_pose_offset_world_frame[1, 0]],
                                                        [stone_pose_offset_world_frame[2, 0]], 
                                                        [0.0], 
                                                        [0.0], 
                                                        [0.0]])
    
    stone_pose = np.add(env_frame_pose, stone_pose_offset_world_frame_extended)
    
    # print('stone_pose: ', stone_pose)
    # print('float(stone_pose[1]): ', float(stone_pose[1]))
    # "pose" : stone_pose,
    stone_yaml = {"name" : "h" + str(0) + "v" + str(0), 
                    "pose" : [float(stone_pose[0]),
                                float(stone_pose[1]),
                                float(stone_pose[2]),
                                float(stone_pose[3]),
                                float(stone_pose[4]),
                                float(stone_pose[5])],
                    "scale" : [plank_platform_scale[0], plank_platform_scale[1], plank_platform_scale[2]],
                    "color" : color } 
    stones.append(stone_yaml)    

    #########################################################################
    ############################ Ending platform ############################
    #########################################################################

    stone_center_pose_env_frame = np.array([[(start_platform_width / 2.0) + (plank_width / 2.0)], #
                                            [0.0],
                                            [-(start_platform_height / 2.0)]])

    stone_pose_offset_world_frame = np.matmul(rot_mat, stone_center_pose_env_frame)
    # print('stone_pose_offset_world_frame: ', stone_pose_offset_world_frame)

    color = get_constant_color()

    stone_pose_offset_world_frame_extended = np.array([[stone_pose_offset_world_frame[0, 0]], 
                                                        [stone_pose_offset_world_frame[1, 0]],
                                                        [stone_pose_offset_world_frame[2, 0]], 
                                                        [0.0], 
                                                        [0.0], 
                                                        [0.0]])
    
    stone_pose = np.add(env_frame_pose, stone_pose_offset_world_frame_extended)
    
    # print('stone_pose: ', stone_pose)
    # print('float(stone_pose[1]): ', float(stone_pose[1]))
    # "pose" : stone_pose,
    stone_yaml = {"name" : "h" + str(num_h_stones + 1) + "v" + str(num_v_stones +1), 
                    "pose" : [float(stone_pose[0]),
                                float(stone_pose[1]),
                                float(stone_pose[2]),
                                float(stone_pose[3]),
                                float(stone_pose[4]),
                                float(stone_pose[5])],
                    "scale" : [start_platform_scale[0], start_platform_scale[1], start_platform_scale[2]],
                    "color" : color } 
    stones.append(stone_yaml)

    env_yaml["models"] = stones

    filename = args.filename
    filepath = os.path.join("/home/masselmeier3/ros2_ws/src/mmp_quadruped/worlds/perceptive/" + filename + ".yaml")

    with open(filepath, 'w') as yaml_file:
        yaml.dump(env_yaml, yaml_file, default_flow_style=False)

    return
