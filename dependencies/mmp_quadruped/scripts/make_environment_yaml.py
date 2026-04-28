import argparse
import os
import yaml
import numpy as np

from utils import rotation_matrix
from stones import make_stepping_stones
from pegboard import make_pegboard
from rubble import make_rubble
from ramp import make_ramp
from stairs import make_stairs
from sparse_stones import make_sparse_stones
from side_stones import make_side_stones

random_seed = 2
np.random.seed(random_seed)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Data Collection Process")
    
    ## Environment flag
    parser.add_argument("-e", "--env", default="none", help="flag for which environment to generate")
    parser.add_argument("-f", "--filename", default="none", help="filename for environment")

    ## stepping stone-specific args
    parser.add_argument("-nhs", "--num_h_stones", default=20, help="number of stones in a row")
    parser.add_argument("-nvs", "--num_v_stones", default=20, help="number of stones in a column")
    parser.add_argument("-xs", "--x_spacing", default=0.15, help="x-dir spacing of stepping stones")    
    parser.add_argument("-ys", "--y_spacing", default=0.15, help="y-dir spacing of stepping stones")    
    parser.add_argument("-zs", "--z_spacing", default=0.0, help="z-dir spacing of stepping stones")    

    parser.add_argument("-sl", "--stone_length", default=0.15, help="stone length")    
    parser.add_argument("-sw", "--stone_width", default=0.15, help="stone width")
    parser.add_argument("-sh", "--stone_height", default=0.10, help="stone height")    

    ## Universal args
    parser.add_argument("-gx", "--env_frame_x", default=0.0, help="x-position of grid origin frame")
    parser.add_argument("-gy", "--env_frame_y", default=0.0, help="y-position of grid origin frame")
    parser.add_argument("-gz", "--env_frame_z", default=0.0, help="z-position of grid origin frame")
    parser.add_argument("-gro", "--env_frame_roll", default=0.0, help="roll of grid origin frame")
    parser.add_argument("-gpi", "--env_frame_pitch", default=0.0, help="pitch of grid origin frame")
    parser.add_argument("-gya", "--env_frame_yaw", default=0, help="yaw of grid origin frame")
    args = parser.parse_args()

    if (args.env == "pegboard"):
        make_pegboard(args)
    elif (args.env == "ramp"):
        make_ramp(args)      
    elif (args.env == "rubble"):
        make_rubble(args)        
    elif (args.env == "sparse_stones"):
        make_sparse_stones(args)      
    elif (args.env == "side_stones"):
        make_side_stones(args)        
    elif (args.env == "stairs"):
        make_stairs(args)        
    # if (args.env == "balance_beam"):
    #     make_stepping_stones(args)
    # elif (args.env == "ramp"):
    #     make_ramp(args)
    else:
        raise Exception(args.env + " has not been implemented yet!")