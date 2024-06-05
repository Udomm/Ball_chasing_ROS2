import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
from .red_ball_tracker import RedballTracker 
# from red_ball_tracker import RedballTracker
import numpy as np
import cv2
import math

class BallChasingNode(Node):
    def __init__(self):
        super().__init__('ball_chasing_node')
        # self.bridge = CvBridge()
        self.sub_ = self.create_subscription(Float32MultiArray, 'ball_position', self.image_callback, 10)
        self.x_center_pub_ = self.create_publisher(Float32, 'x_center', 10)
        self.area_pub_ = self.create_publisher(Float32, 'area', 10)
        

    def image_callback(self, data):
        
        data = data.data
        if len(data) == 4:
            x1, y1, x2, y2 = data

            x_center = self.x_center(x1, x2)

            area = self.area(x1, x2, y1, y2)

            x_center_msgs = Float32()

            x_center_msgs.data = x_center


        

        


        

            self.x_center_pub_.publish(x_center_msgs) # Publish the position of ball
            self.area_pub_.publish(area) # Publish the position of ball
        # processed_msg = self.bridge.cv2_to_imgmsg(squeezed_arr, "bgr8") # Convert the processed OpenCV image back to ROS Image message
        # self.image_pub_.publish(processed_msg) # Publish the processed image
        else: 
            return None
    def x_center(self, x1, x2):
        x_center = (x1+x2)/2
        return x_center
    
    def area(seld, x1, x2, y1, y2):
        area = (x2-x1)*(y2-y1)
        return area


def main(args=None):
    rclpy.init(args=args)
    ball_chasing_node = BallChasingNode()
    rclpy.spin(ball_chasing_node)
    ball_chasing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

