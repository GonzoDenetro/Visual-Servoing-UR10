import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
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
        self.pose_publisher = self.create_publisher()
        
        #Aruco Detector Configuration
        self.aruco_dict = cv.aruco.getPredifinedDictrionary(cv.aruco.DICT_6X6_250) #Dictionary of Aruco markers of 6x6 bits
        self.parameters = cv.aruco.DetectorParams()
        self.detector = cv.aruco.ArucoDetector(self.aruco_dict)
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
            [600.0,   0.0, 320.0],
            [  0.0, 600.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float64)
        self.D_matrix = np.zeros((5, 1), dtype=np.float64)
        #Bridge for OpenCV and ROS2
        self.bridge = CvBridge()
        


    def pose_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        #Detect Aruco Markers (returns the coordinates of the corners and the ids)
        corners, ids, _ = self.detector.detectMakers(img_gray)
        detected_msg = Bool()
        
        #Markers not detected
        if ids is None:
            detected_msg.data = False
            return
        
        #Pose Estimation (PnP)
        for marker_corners in corners:
            image_points = marker_corners[0].astype(np.float64)
            
            #Solve PNP (return: succces, rotation vector and translation vector)
            succes, rotation_vector, translation_vector = cv.solvePnP(
                self.marker_3d_points,
                image_points,
                self.k_matrix,
                self.D_matrix,
                flags=cv.SOLVEPNP_IPPE_SQUARE  # More precision for plane markers
            )         
        
    
    def camera_info_callback(self, msg):
        if self._user_camera_info:
            return
        
        # Get the intrinsic parameters of the camera, from camera_info/
        self.k_matrix = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D_matrix = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self._user_camera_info = True

        self.get_logger().info("Intrinsic Matrix Ready")
        
    

def main(args=None):
    rclpy.init(args=args) #Initialze Communiation
    
    #Node event loop
    node = PoseEstimationNode()
    rclpy.spin(node)
    
    rclpy.shutdown() #End Communiaation
    

if __name__ == '__main__':
    main()
        