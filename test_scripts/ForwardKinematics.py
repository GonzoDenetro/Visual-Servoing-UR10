import numpy as np

 # DH Parameters
    #   | tetha         | alpha    | a      | d
    # 1 | theta_1       |   0      | 0      | 0
    # 2 | theta_2 + 90° |   90°    | 0      | 0
    # 3 | theta_3       |   0      | a_2    | 0
    # 4 | theta_4 - 90° |   0      | a_3    | d_4
    # 5 | theta_5       |   -90°   | 0      | d_5
    # 6 | theta_6       |   90°    | 0      | 0


def transformationMatrix(tetha, alpha, a, d):
    matrix = np.array([
        [np.cos(tetha), -np.sin(tetha)*np.cos(alpha), np.sin(tetha)*np.sin(alpha), a*np.cos(tetha)],
        [np.sin(tetha), np.cos(tetha)*np.cos(alpha), -np.cos(tetha)*np.sin(alpha), a*np.sin(tetha)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
    return matrix

def forwardKinematics(joint_angles, link_lengths):
    
    # Joint Angles and Linj lengths Variables
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = joint_angles
    a_2, a_3, d_1, d_4, d_5, d_6 = link_lengths
    #d_1, d_4, d_5, d_6 = 0.1807, 0.17415, 0.12, 0.11655
    #a_2, a_3          = -0.6127, -0.5716  # negativos en la convención UR
    
    
    #Transformation Matrices
    T_01 = transformationMatrix(theta_1, np.pi/2, 0, d_1)
    T_12 = transformationMatrix(theta_2, 0, a_2, 0)
    T_23 = transformationMatrix(theta_3, 0, a_3, 0)
    T_34 = transformationMatrix(theta_4, np.pi/2, 0, d_4)
    T_45 = transformationMatrix(theta_5, -np.pi/2, 0, d_5)
    T_56 = transformationMatrix(theta_6, 0, 0, d_6)
    
    # Matrix Multiplication
    T_02 = T_01 @ T_12
    T_03 = T_02 @ T_23
    T_04 = T_03 @ T_34
    T_05 = T_04 @ T_45
    T_06 = T_05 @ T_56
    
    return T_06, [T_01, T_02, T_03, T_04, T_05]


def run():
    joint_angles = [0, -np.pi/2, 0, 0, 0, 0]
    link_lengths = [-0.612, -0.572, 0.12, 0.16, 0.115, 0.0922]
    
    result, tranformations = forwardKinematics(joint_angles, link_lengths)
    # Ajustar decimales y suprimir notación científica
    np.set_printoptions(precision=3, suppress=True)

    print("End Effector Pose:\n", result)
    print("------------------------------")
    print(result[:3, 3:])


if __name__ == "__main__":
    run()