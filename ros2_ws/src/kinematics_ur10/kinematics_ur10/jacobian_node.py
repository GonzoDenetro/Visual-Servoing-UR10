import rclpy  
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JacobianNode(Node):
    def __init__(self):
        super().__init__("Jacobian")
        

def main(args=None):
    rclpy.init() #Initialze Comunication
    
    #Node event loop
    node = JacobianNode()
    rclpy.spin(node)
    
    rclpy.shutdown() #End Comunication