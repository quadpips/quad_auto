import lxml.etree as ET
import yaml
import numpy as np
import json
import os
import argparse
# import rospkg
# rospack = rospkg.RosPack()

# Define the main content and content to be added as strings
main_content_str = """
<sdf version='1.7'>
  <world name='default'>

    <!-- Need to be able to manipulate model state -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>

      <update_rate>1.0</update_rate>
    </plugin>

    <!-- Need to be able to manipulate model configuration -->
    <plugin name="gazebo_ros_properties" filename="libgazebo_ros_properties.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      
      <update_rate>1.0</update_rate>
    </plugin>

    <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.81</gravity>
        <ode>
            <solver>
            <type>quick</type>  
            <iters>50</iters> 
            <sor>1.3</sor>
            </solver>  
            <constraints>
            <cfm>0.0</cfm>
            <erp>0.2</erp>
            <contact_max_correcting_vel>10.0</contact_max_correcting_vel>
            <contact_surface_layer>0.001</contact_surface_layer>
            </constraints>  
        </ode>
    </physics>
    
    <!-- A global light source -->
    <include>
        <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
   
    <model name="static_environment">
    <static>true</static>

    </model>
  </world>
</sdf>
"""

# <include>
#       <uri>model://ground_plane</uri>
# </include> 

def quaternion_to_euler_angle(x, y, z, w):
    print("quaternion_to_euler_angle: ")
    print(" x: ", x)
    print(" y: ", y)
    print(" z: ", z)
    print(" w: ", w)
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1) # np.degrees()

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2) # np.degrees(

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4) # np.degrees(

    print(" X: ", X)
    print(" Y: ", Y)
    print(" Z: ", Z)

    return X, Y, Z 

def generate_steppable_env(main_content, models_data, mu):
    # Find the specific location to insert the additional content (under state element)
    # state_element1 = main_content.find(".//state[@world_name='default']")
    
    state_element2 = main_content.find(".//model[@name='static_environment']")
    for idx, model_data in enumerate(models_data["models"]):
        model_name = model_data["name"]
        pose_values = model_data["pose"]
        if ("bar_v" in model_name): # ad-hoc fix to have vertical bars scaled in y direction
            orig_scale_values = model_data["scale"]
            scale_values = [orig_scale_values[1], orig_scale_values[0], orig_scale_values[2]]
        else:
            scale_values = model_data["scale"]

        color = model_data["color"]

        additional_content_str2 = f"""
            <link name="region_{idx}">
            <pose>{" ".join(map(str, pose_values))}</pose>
            <collision name="region_{idx}_collision">
                <geometry>
                    <box>
                        <size>{" ".join(map(str, scale_values))}</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                    <ode>
                        <mu>{str(mu)}</mu>
                        <mu2>{str(mu)}</mu2>
                    </ode>
                    </friction>
                </surface>                
            </collision>
            <visual name="region_{idx}_visual">
                <geometry>
                    <box>
                        <size>{" ".join(map(str, scale_values))}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>{str(color[0])} {str(color[1])} {str(color[2])} 1.0</ambient>
                    <diffuse>{str(color[0])} {str(color[1])} {str(color[2])} 1.0</diffuse>
                </material>
            </visual>
            </link>
            """.format(idx=idx)

        state_element2.append(ET.fromstring(additional_content_str2))
        ET.indent(state_element2, '  ') #  have each link start on a new line

    return main_content

# Reads in .yaml
def generate_world(file_name, models_data, mu):
    # Parse the main content as XML
    main_content = ET.fromstring(main_content_str)

    main_content = generate_steppable_env(main_content, models_data, mu)

    # Serialize the modified XML back to a string
    modified_content_str = ET.tostring(main_content, pretty_print=True, encoding="unicode")

    # Print or save the modified content as needed
    # output_file_path = "./" + file_name + ".world"

    # path = rospack.get_path("mmp_quadruped")
    path = "/home/masselmeier3/ros2_ws/src/mmp_quadruped/"

    output_file_path = os.path.join(path, "worlds/perceptive/", file_name + ".world")

    # Write the modified content to the output file
    with open(output_file_path, "w") as output_file:
        output_file.write(modified_content_str)

def generate_stones_json(file_name, models_data):

    # Output json file for region
    poses = []
    scales = []
    colors = []
    for idx, model_data in enumerate(models_data["models"]):
        pose_values = model_data["pose"]
        scale_values = model_data["scale"]
        color_values = model_data["color"]

        # stone_pose = np.array([float(pose_values[0]), 
        #                      float(pose_values[1]), 
        #                      float(pose_values[2]),
        #                      float(pose_values[3]),
        #                      float(pose_values[4]),
        #                      float(pose_values[5])])

        # stone_scale = np.array([float(scale_values[0]),
        #                         float(scale_values[1]),
        #                         float(scale_values[2])])

        # grid_frame_roll = float(pose_values[3])
        # grid_frame_pitch = float(pose_values[4])
        # grid_frame_yaw = float(pose_values[5])
        
        # R11 = np.cos(grid_frame_yaw)*np.cos(grid_frame_pitch)
        # R12 = np.cos(grid_frame_yaw)*np.sin(grid_frame_pitch)*np.sin(grid_frame_roll)-np.sin(grid_frame_yaw)*np.cos(grid_frame_roll)
        # R13 = np.cos(grid_frame_yaw)*np.sin(grid_frame_pitch)*np.cos(grid_frame_roll)+np.sin(grid_frame_yaw)*np.sin(grid_frame_roll)
        # R21 = np.sin(grid_frame_yaw)*np.cos(grid_frame_pitch)
        # R22 = np.sin(grid_frame_yaw)*np.sin(grid_frame_pitch)*np.sin(grid_frame_roll)+np.cos(grid_frame_yaw)*np.cos(grid_frame_roll)
        # R23 = np.sin(grid_frame_yaw)*np.sin(grid_frame_pitch)*np.sin(grid_frame_roll)-np.cos(grid_frame_yaw)*np.sin(grid_frame_roll)
        # R31 = -np.sin(grid_frame_pitch)
        # R32 = np.cos(grid_frame_pitch)*np.sin(grid_frame_roll)
        # R33 = np.cos(grid_frame_pitch)*np.cos(grid_frame_roll)

        # T = np.array([[R11, R12, R13, stone_pose[0, 0]],
        #               [R21, R22, R23, stone_pose[1, 0]],
        #               [R31, R32, R33, stone_pose[2, 0]],
        #               [0.0, 0.0, 0.0, 1.0]])

   
        # # top face, bottom left point
        # disp_vec_top_face_bottom_left_pt_stone_frame = np.array([[-float(scale_values[0]) / 2.0],
        #                                                             [-float(scale_values[1]) / 2.0],
        #                                                             [float(scale_values[2]) / 2.0],
        #                                                             [1.0]])    
        
        # # top face, bottom right point
        # disp_vec_top_face_bottom_right_pt_stone_frame = np.array([[float(scale_values[0]) / 2.0],
        #                                                             [-float(scale_values[1]) / 2.0],
        #                                                             [float(scale_values[2]) / 2.0],
        #                                                             [1.0]])   

        # # top face, top right point
        # disp_vec_top_face_top_right_pt_stone_frame = np.array([[float(scale_values[0]) / 2.0],
        #                                                         [float(scale_values[1]) / 2.0],
        #                                                         [float(scale_values[2]) / 2.0],
        #                                                         [1.0]])   

        # origin_grid_frame = np.array([[0.0],
        #                               [0.0],
        #                               [0.0],
                                    #   [1.0]])   
        # stone_normal_grid_frame = np.array([[0.0],
        #                                     [0.0],
        #                                     [1.0],
        #                                     [1.0]])   

        # bot_left_pt = np.matmul(T, disp_vec_top_face_bottom_left_pt_stone_frame)
        # bot_right_pt = np.matmul(T, disp_vec_top_face_bottom_right_pt_stone_frame)
        # top_right_pt = np.matmul(T, disp_vec_top_face_top_right_pt_stone_frame)

        # grid_origin_world_frame = np.matmul(T, origin_grid_frame)
        # stone_normal = np.matmul(T, stone_normal_grid_frame) - grid_origin_world_frame
        # disp_vec_grid_frame = np.array([[float(scale_values[0]) / 2.0],
        #                                 [float(scale_values[1]) / 2.0]])
        # disp_vec_world_frame = np.matmul(rot_mat, disp_vec_grid_frame)
        # print('disp_vec_world_frame: ', disp_vec_world_frame)

        # # get first end point
        # #   transform using stone pose
        
        # endpt1 = stone_pose - np.array([disp_vec_world_frame[0, 0], disp_vec_world_frame[1, 0], 0.0])
        # print('endpt1: ', endpt1)

        # # get second end point
        # #   transform using stone pose
        # endpt2 = stone_pose + np.array([disp_vec_world_frame[0, 0], disp_vec_world_frame[1, 0], 0.0])
        # print('endpt2: ', endpt2)    

        # print('top_right_pt: ', top_right_pt)

        # print('bot_left_pt: ', bot_left_pt)

        # stone_endpts = [[bot_left_pt[0, 0], bot_left_pt[1, 0], bot_left_pt[2, 0]],
        #                 [bot_right_pt[0, 0], bot_right_pt[1, 0], bot_right_pt[2, 0]], 
        #                 [top_right_pt[0, 0], top_right_pt[1, 0], top_right_pt[2, 0]]]
        # endpoints.append(stone_endpts)
        poses.append(pose_values)
        scales.append(scale_values)
        colors.append(color_values)

    contents = {"poses" : poses,
                "scales" : scales,
                "colors" : colors}
    stone_json = {"stone_setup" : contents }

    # Serializing json
    json_object = json.dumps(stone_json, indent=4)

    # Writing to sample.json
    path = "/home/masselmeier3/ros2_ws/src/mmp_quadruped/" 
    # rospack.get_path("mmp_quadruped")
    

    filepath = os.path.join(path, "config/environment/perceptive/", file_name + ".json")
    with open(filepath, "w") as outfile:
        outfile.write(json_object)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Data Collection Process")
    
    ## Environment flag
    parser.add_argument("-e", "--env", default="stepping_stone", help="flag for which environment to generate")
    parser.add_argument("-f", "--filename", default="placeholder", help="filename for environment")
    parser.add_argument("-c", "--friction_coefficient", default=0.5, help="friction coefficient for terrain")

    args = parser.parse_args()
    
    filename = args.filename
    mu = args.friction_coefficient

    # path = rospack.get_path("mmp_quadruped")
    path = "/home/masselmeier3/ros2_ws/src/mmp_quadruped/"

    # file_name = args.env
    with open(path + "/worlds/perceptive/" + filename + ".yaml", "r") as yaml_file:
        models_data = yaml.safe_load(yaml_file)
    generate_world(filename, models_data, mu)

    generate_stones_json(filename, models_data)
