import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("Forward_kinematics")
        self.get_logger().info('Forward Kinematics Node has been started')
        
        # D-H Parameters (UR10)
        self.dh_params = [
            ( np.pi / 2,   0.0,     0.1270),   # joint 1
            ( 0.0,        -0.612,  0.0   ),   # joint 2
            ( 0.0,        -0.572,  0.0   ),   # joint 3
            ( np.pi / 2,   0.0,     0.1639),   # joint 4
            (-np.pi / 2,   0.0,     0.1157),   # joint 5
            ( 0.0,         0.0,     0.0922),   # joint 6
        ]
                
        #Create Subscriber
        self.joint_subscriber_ = self.create_subscription(JointState, '/joint_states', self.forward_callback, 10)
        
        #Create Publisher
        #self.transformations_publisher_ = self.create_publisher()
            
    
    def forward_callback(self, msg):
        joints = list(msg.position)
        
        T_06, T_intermediate = self.forward_kinematics(joints)
        
        np.set_printoptions(suppress=True, precision=3)        
        self.get_logger().info(f"T_06:\n{T_06.round(3)}")
    
    def forward_kinematics(self, joint_angles):
        T_list = []
        T_matrix_prev = np.eye(4)
        
        for i, (theta_i, params) in enumerate(zip(joint_angles, self.dh_params)):
            alpha_i, a_i, d_i = params
            
            T_matrix = self.transformation_matrix(theta_i, alpha_i, a_i, d_i) #Homogeneus Transformation Matrix
            T_mul = T_matrix_prev @ T_matrix #Matrix Multiplication
            
            if i < 5:
                T_list.append(T_mul.copy())
                
            T_matrix_prev = T_mul
        
        return T_mul, T_list
        
    
    
    def transformation_matrix(self, tetha, alpha, a, d):
        matrix = np.array([
            [np.cos(tetha), -np.sin(tetha)*np.cos(alpha), np.sin(tetha)*np.sin(alpha), a*np.cos(tetha)],
            [np.sin(tetha), np.cos(tetha)*np.cos(alpha), -np.cos(tetha)*np.sin(alpha), a*np.sin(tetha)],
            [0.0, np.sin(alpha), np.cos(alpha), d],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
        return matrix
        


def main(args=None):
    rclpy.init(args=args) #Inicializar Comunicación 
    
    #Nodo
    node = ForwardKinematicsNode()
    rclpy.spin(node) # Event Loop
    
    rclpy.shutdown() # Terminar Comunicación
    

if __name__ == "__main__":
    main()