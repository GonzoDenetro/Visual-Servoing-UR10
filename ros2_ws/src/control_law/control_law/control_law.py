import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class ControlLaw(Node):
    def __init__(self):
        pass


def main(args=None):
    rclpy.init() #Initialze Communication
    
    #Event Loop Node
    node = ControlLaw()
    rclpy.spin(node)
    
    #End Communication
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()