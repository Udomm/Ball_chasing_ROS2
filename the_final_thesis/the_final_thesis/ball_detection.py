import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from cv_bridge import CvBridge, CvBridgeError
from .red_ball_tracker import RedballTracker 
# from red_ball_tracker import RedballTracker
import numpy as np
import cv2

class RedballDetectionNode(Node):
    def __init__(self):
        super().__init__('redball_tracker_node')
        self.bridge = CvBridge()
        self.sub_ = self.create_subscription(Image, 'camera_topic', self.image_callback, 10)
        self.ball_pos_pub_ = self.create_publisher(Float32MultiArray, 'ball_position', 10)
        # self.image_pub_ = self.create_publisher(Image, 'processed_image', 10)
        

    def image_callback(self, frame):
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, "bgr8") # Convert ROS Image message to OpenCV image

            ball_position = self.process_frame(frame)  # Process the images
            # output_video_frames = cv2.cvtColor(output_video_frames, cv2.COLOR_BGR2BGRA)
            # ball_position_msg = ball_position
            # squeezed_arr = np.squeeze(output_video_frames[0])


            ball_position_msg = Float32MultiArray()

            ball_position_msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="example", size=4, stride=4)], data_offset=0)
            ball_position_msg.data = ball_position

            self.ball_pos_pub_.publish(ball_position_msg) # Publish the position of ball

            # processed_msg = self.bridge.cv2_to_imgmsg(squeezed_arr, "bgr8") # Convert the processed OpenCV image back to ROS Image message
            # self.image_pub_.publish(processed_msg) # Publish the processed image


        except CvBridgeError as e:
            self.get_logger().error("Could not convert image to 'bgr8'. Error: %s" % e)
    
    def process_frame(self, frame):
        redball_tracker = RedballTracker(model_path='/home/udom/dev_ws/src/the_final_thesis/the_final_thesis/models/best.pt')
        
        # Ensure frame is a 3D array (height, width, channels)
        if frame.ndim == 2:
            frame = np.expand_dims(frame, axis=-1)  # Add channel dimension
        if frame.ndim == 2:
            frame = np.expand_dims(frame, axis=-1)  # Add channel dimension
        # Ensure frame is a 4D array (batch, height, width, channels)
        if frame.ndim == 3:  # Single frame case
            frame = np.expand_dims(frame, axis=0)  # Add batch dimension
        

        ball_detections = redball_tracker.detect_frames(frame)

        ball_position = [value for d in ball_detections for key, values in d.items() for value in values]
        ball_position = ball_position[:4]        
        print("ball_position", ball_position)
        print("ball_detections", ball_detections[0])
        ## Draw Ball Bounding Boxes
        # output_video_frames = redball_tracker.draw_bboxes(frame, ball_detections[0])
        # output_video_frames = np.array(output_video_frames)
        return ball_position

def main(args=None):
    rclpy.init(args=args)
    redball_tracker_node = RedballDetectionNode()
    rclpy.spin(redball_tracker_node)
    redball_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

