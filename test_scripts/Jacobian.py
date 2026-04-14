import numpy as np
from ForwardKinematics import *

#Only for Revolute Joints
def get_Jacobian(tranformations, T_b_ee):
    identity = np.eye(4)
    T_list = [identity] + list(tranformations)
    z_hat = np.array([0, 0, 1]).reshape(-1, 1)
    n_joints = len(T_list) - 1
    jacobian = np.zeros((6, n_joints))
    
    for i in range(n_joints): 
        R_i = T_list[i][:3, :3] #Rotation of frame i
        p_i = T_list[i][:3, [3]] #Position of frame i
        p_ee = T_b_ee[:3, 3:] #Position End-Effector
        
        #Angular Part
        angular_array = R_i @ z_hat

        #Linear Part
        distance_difference = p_ee - p_i
        linear_array = np.cross(angular_array.flatten(), distance_difference.flatten()).reshape(-1, 1)
        
        jacobian[:3, i] = linear_array.flatten()
        jacobian[3:, i] = angular_array.flatten()
        
    return jacobian



#UR10 Test
joint_angles = [0, -np.pi/2, 0, 0, 0, 0]
link_lengths = [-0.613, -0.572, 0.181, 0.174, 0.120, 0.117]

T_b_ee, transformations = forwardKinematics(joint_angles, link_lengths)

J = get_Jacobian(transformations, T_b_ee=T_b_ee)

np.set_printoptions(suppress=True, precision=3)
print(f"K:\n{J.round(3)}")