import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('Pose_estimation')

        #Create Subscriber for image
        self.img_subscriber = self.create_subscription(Image, '/wrist_camera/image_raw', self.pose_callback, 10)


    def pose_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args) #Initialze Communiation
    
    #Node event loop
    node = PoseEstimationNode()
    rclpy.spin(node)
    
    rclpy.shutdown() #End Communiaation
    

if __name__ == '__main__':
    main()
        