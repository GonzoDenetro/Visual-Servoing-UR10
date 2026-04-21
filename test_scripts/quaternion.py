import numpy as np

def get_quaternion(matrix):
    R = matrix[:3, :3] #3x3
    trace = R[0,0] + R[1,1] + R[2,2] #Sum of the diagonal elements of the matrix 
        
    if trace > 0:
        s = 2.0 * np.sqrt(1+trace)
        q_0 = 0.25 * s 
        q_1 = (R[2, 1] - R[1, 2]) / s
        q_2 = (R[0, 2] - R[2, 0]) / s
        q_3 = (R[1, 0] - R[0, 1]) / s
            
    elif R[0, 0] > R[1, 1] and R[0,0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q_0 = (R[2, 1] - R[1, 2]) / s
        q_1 = 0.25 * s
        q_2 = (R[1, 0] + R[0, 1]) / s
        q_3 = (R[0, 2] + R[2, 0]) / s
        
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q_0 = (R[0, 2] - R[2, 0]) / s
        q_1 = (R[0, 1] + R[1, 0]) / s
        q_2 = 0.25 * s
        q_3 = (R[1, 2] + R[2, 1]) / s
            
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q_0 = (R[1, 0] - R[0, 1]) / s
        q_1 = (R[0, 2] + R[2, 0]) / s
        q_2 = (R[1, 2] + R[2, 1]) / s
        q_3 = 0.25 * s
    
    return q_0, q_1, q_2, q_3

matrix =  np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1], 
])

quaternion = get_quaternion(matrix)

print(quaternion)