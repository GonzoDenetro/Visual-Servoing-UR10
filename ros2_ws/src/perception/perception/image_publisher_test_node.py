import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


class ImagePublusher(Node):
    def __init__(self):
        super().__init__('Image_publihser')
        self.path = '/home/gonzcode/Robotics/VisualServoing/test_scripts/Assets/Aruco_test.jpg'
        
        # Create Image Publisher
        self.image_publisher_ = self.create_publisher(Image, '/wrist_camera/image_raw', 10)
        
        self.timer = self.create_timer(0.5, self.image_callback)
        self.frame = cv.imread(self.path)
        
        if self.frame is None:
            self.get_logger().error("No se pudo cargar la imagen")
        else:
            self.get_logger().info("Imagen cargada correctamente")

        self.bridge = CvBridge()
        
    
    def image_callback(self):
        if self.frame is None:
            return
        
        img_msg = self.bridge.cv2_to_imgmsg(self.frame)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'wrist_camera_link'
        
        self.image_publisher_.publish(img_msg)
        self.get_logger().info('Image Published')

def main(args=None):
    rclpy.init()
    
    node = ImagePublusher()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()