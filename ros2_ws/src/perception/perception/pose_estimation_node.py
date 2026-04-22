import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2 as cv
import numpy as np


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('Pose_estimation')

        #Create Subscriber for image
        self.img_subscriber = self.create_subscription(Image, '/wrist_camera/image_raw', self.pose_callback, 10)
        
        #Create a Subscriber for the camera info (to get intrinsic matrix)
        self.cam_info_subscriber = self.create_subscription(CameraInfo, '/wrist_camera/camera_info', self.camera_info_callback, 10)
        
        #Create Pose Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/ur10/aruco_pose', 10)
        
        #Detection detected Publisher
        self.detected_publisher = self.create_publisher(Bool, '/ur10/aruco_detected', 10)
        
        #Aruco Detector Configuration
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250) #Dictionary of Aruco markers of 6x6 bits
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.marker_length = 0.0104
        
        self.marker_3d_points = np.array([
            [-self.marker_length/2, self.marker_length/2, 0.0],
            [self.marker_length/2, self.marker_length/2, 0.0],
            [self.marker_length/2, -self.marker_length/2, 0.0],
            [-self.marker_length/2, -self.marker_length/2, 0.0]
        ], dtype=np.float64)
        
        #Intrinsic Matrix
        self._user_camera_info = False
        self.k_matrix = np.array([
            [3149.439,   0.0, 227.376],
            [  0.0, 4079.515, 210.289],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float64)
        #self.D_matrix = np.zeros((5, 1), dtype=np.float64)
        self.D_matrix = np.array([[ 11.303 -364.221   -0.038   -0.357   -6.173]])
        #Bridge for OpenCV and ROS2
        self.bridge = CvBridge()


    def pose_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        #Detect Aruco Markers (returns the coordinates of the corners and the ids)
        corners, ids, _ = self.detector.detectMarkers(img_gray)
        detected_msg = Bool()
        
        #Markers not detected
        if ids is None:
            detected_msg.data = False
            return
        
        #Pose Estimation (PnP)
        for marker_corners in corners:
            image_points = marker_corners[0].astype(np.float64)
            
            #Solve PNP (return: succces, rotation vector and translation vector) #Inthe camera reference system
            success, rotation_vector, translation_vector = cv.solvePnP(
                self.marker_3d_points,
                image_points,
                self.k_matrix,
                self.D_matrix,
                flags=cv.SOLVEPNP_IPPE_SQUARE  # More precision for plane markers
            )         
            
            
            
            if success:
                
                #The rotation vector comes in the axis_angle representatio, we use Rodrigues formula 
                # to pass it to Rotation matrix
                R, _ = cv.Rodrigues(rotation_vector)
                
                np.set_printoptions(suppress=True, precision=3)
                self.get_logger().info(f"Rotation Matrix:\n{R.round(3)}")
                
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = translation_vector.flatten()
                
                #To publish the pose to the control law, the type of message is PoseStamped,
                # the orientation in PoseStamped is represented in quaternions, we transform it
                image_time_stamp = msg.header.stamp
                pose_msg = self.matrix_to_PoseStamped(T, image_time_stamp, frame_id='wrist_camera_link')
                self.pose_publisher.publish(pose_msg)
                
                detected_msg.data = True
                self.detected_publisher.publish(detected_msg)
    
    def camera_info_callback(self, msg):
        if self._user_camera_info:
            return
        
        # Get the intrinsic parameters of the camera, from camera_info/
        self.k_matrix = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D_matrix = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self._user_camera_info = True

        self.get_logger().info("Intrinsic Matrix Ready")
        
    def matrix_to_PoseStamped(self, matrix, time_stamp, frame_id):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = time_stamp
        pose_stamped_msg.header.frame_id = frame_id #Coordinates reference system
        
        #Position
        pose_stamped_msg.pose.position.x = matrix[0, 3]
        pose_stamped_msg.pose.position.y = matrix[1, 3]
        pose_stamped_msg.pose.position.z = matrix[2, 3]
        
        #Orientation
        quaternions = self.rotation_to_quaternion(matrix)
        pose_stamped_msg.pose.orientation.x = quaternions[1]
        pose_stamped_msg.pose.orientation.y = quaternions[2]
        pose_stamped_msg.pose.orientation.z = quaternions[3]
        pose_stamped_msg.pose.orientation.w = quaternions[0] 
        
        return pose_stamped_msg
        
    def rotation_to_quaternion(self, matrix):
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

        self.get_logger().info(f"Quaternion:\n w:{q_0}, x:{q_1}, y:{q_2}, z:{q_3}")
        
        return q_0, q_1, q_2, q_3 

def main(args=None):
    rclpy.init(args=args) #Initialze Communiation
    
    #Node event loop
    node = PoseEstimationNode()
    rclpy.spin(node)
    
    rclpy.shutdown() #End Communiaation
    

if __name__ == '__main__':
    main()
        