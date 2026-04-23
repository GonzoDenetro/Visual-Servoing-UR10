import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np


class ControlLaw(Node):
    def __init__(self):
        self.T_current_pose = None
        self.jacobian = None
        self.marker_detected = False
        
        #Subscribers
        self.jacobian_subscriber_ = self.create_subscription(Float64MultiArray, '/ur10/jacobian', self.jacobian_callback, 10)
        self.pose_subscriber_ = self.create_subscription(PoseStamped, '/ur10/aruco_pose', self.pose_callback, 10)
        self.detected_subscriber = self.create_subscription(Bool, '/ur10/aruco_detected', self.detected_callback, 10)
        
        #Joint Velocitiies Publisher
        
    
    def  jacobian_callback(self, msg):
        data = np.array(msg.data)
        
        if data.shape[0] != 36:
            self.get_logger().info('Jacobians comes in incorrect size')
            return
        
        self.jacobian = data.reshape(6, 6) #Matrix 6x6
        
    
    def pose_callback(self, msg):
        pass
    
    
    def detected_callback(self, msg):
        self.marker_detected = msg.data
    
    

def main(args=None):
    rclpy.init() #Initialze Communication
    
    #Event Loop Node
    node = ControlLaw()
    rclpy.spin(node)
    
    #End Communication
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()