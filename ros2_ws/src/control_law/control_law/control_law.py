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
        
        # Control Parameters
        self.alpha = 0.1
        
        #Desire Pose
        self.T_desire = np.array([
            [1.0, 0.0, 0.0,  0.0 ],
            [0.0, 1.0, 0.0,  0.0 ],
            [0.0, 0.0, 1.0,  0.30],   # 30 cm frente a la cámara
            [0.0, 0.0, 0.0,  1.0 ],
        ], dtype=np.float64)
        
        #Subscribers
        self.jacobian_subscriber_ = self.create_subscription(Float64MultiArray, '/ur10/jacobian', self.jacobian_callback, 10)
        self.pose_subscriber_ = self.create_subscription(PoseStamped, '/ur10/aruco_pose', self.pose_callback, 10)
        self.detected_subscriber = self.create_subscription(Bool, '/ur10/aruco_detected', self.detected_callback, 10)
        
        #Joint Velocitiies Publisher
        
        
        #Timer 100Hz
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def  jacobian_callback(self, msg):
        data = np.array(msg.data)
        
        if data.shape[0] != 36:
            self.get_logger().info('Jacobians comes in incorrect size')
            return
        
        self.jacobian = data.reshape(6, 6) #Matrix 6x6
        
    
    def pose_callback(self, msg):
        #Pose
        p = msg.pose.position
        q = msg.pose.orientation
        
        #Rotation Martix
        R = self.quaternion_to_rotation(q)
        
        #Transformation Matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [p.x, p.y, p.z]
        
        self.T_current_pose = T
    
    
    def detected_callback(self, msg):
        self.marker_detected = msg.data
    
    
    def control_loop(self):
        # In the control loop we are going to get set the joint velocities based on the error 
        # of the current pose and desire pose.
        
        # 1.-    -----Get diference between desire pose and current pose
        # 
        # T_c*_c: Is the transformation of the actual camara pose view from the desire camera pose
        # T_c*_c is the equivalent to e = s* - s
        
        T_c*_c = self.T_desire @ np.linalg.inv(self.T_current_pose)
    
    def quaternion_to_rotation(self, quaternion):
        #COnvert quaternions to a rotation matrix of 3x3
        q0, q1, q2, q3 = quaternion
        R = [
            [(2.0*(q0**2 + q1**2))-1, 2.0*(q1*q2 - q0*q3), 2.0*(q1*q3 + q0*q2)],
            [2.0*(q1*q2 + q0*q3), (2.0*(q0**2 + q2**2))-1, 2.0*(q1*q3 - q0*q1)],
            [2.0*(q1*q3 - q0*q2), 2.0*(q2*q3 + q0*q1), (2.0*(q0**2 + q3**2))-1]
        ]
        return R
        
        
def main(args=None):
    rclpy.init() #Initialze Communication
    
    #Event Loop Node
    node = ControlLaw()
    rclpy.spin(node)
    
    #End Communication
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()