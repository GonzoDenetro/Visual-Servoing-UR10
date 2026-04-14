import rclpy  
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np


class JacobianNode(Node):
    def __init__(self):
        super().__init__("Jacobian")
        self.jacobian = 0
        
        #Create Subscriber
        # we create the "/ur10/transforms"
        self.transforms_subscriber_ = self.create_subscription(Float64MultiArray, '/ur10/transforms', self.jacobian_callback, 10)
        
        #Create Publisher
        self.jacobian_publisher_ = self.create_publisher(Float64MultiArray, '/ur10/jacobian', 10)
    
    def jacobian_callback(self, msg):
        matrix = np.array(msg.data)
        
        #Matrix Reconstruction
        matrices = matrix.reshape(6, 4, 4) # 6 matrices of 4x4
        T_intermediate = [matrices[i] for i in range(5)]
        T_06 = matrices[5]
        
        #Get Jacobian
        self.jacobian = self.get_jacobian(T_intermediate, T_06)
        
        #Publish Jacobian
        J_msg = Float64MultiArray()
        J_msg.data = self.jacobian.flatten().tolist()
        self.jacobian_publisher_.publish(J_msg)
        
        #Print Jacobian
        np.set_printoptions(suppress=True, precision=3)
        self.get_logger().info(f"J:\n{self.jacobian.round(3)}")
        
    def get_jacobian(self, transformations, T_b_ee):
        identity = np.eye(4)
        T_list = [identity] + list(transformations)
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
        
        

def main(args=None):
    rclpy.init() #Initialze Comunication
    
    #Node event loop
    node = JacobianNode()
    rclpy.spin(node)
    
    rclpy.shutdown() #End Comunication