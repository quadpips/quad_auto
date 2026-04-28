import numpy as np

def rotation_matrix(roll, pitch, yaw):
    R11 = np.cos(yaw)*np.cos(pitch)
    R12 = np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(roll)
    R13 = np.cos(yaw)*np.sin(pitch)*np.cos(roll)+np.sin(yaw)*np.sin(roll)
    R21 = np.sin(yaw)*np.cos(pitch)
    R22 = np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(roll)
    R23 = np.sin(yaw)*np.sin(pitch)*np.sin(roll)-np.cos(yaw)*np.sin(roll)
    R31 = -np.sin(pitch)
    R32 = np.cos(pitch)*np.sin(roll)
    R33 = np.cos(pitch)*np.cos(roll)

    rot_mat = np.array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])
    
    return rot_mat