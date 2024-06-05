import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('camera_topic is being published')
        else: 
            self.get_logger().error('Unable to open camera')

        timer_period = 0.0083  # seconds (120 frames per second)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            flipped_frame = cv2.flip(frame, 1)
            image_message = self.bridge.cv2_to_imgmsg(flipped_frame, encoding="bgr8")
            self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

