import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()
        self.target_hsv_min_ = self.declare_parameter("target_hue_min", 0.0).value
        self.target_hsv_max_ = self.declare_parameter("target_hue_max", 30.0).value
        self.calibration_in_progress_ = False
        self.sub_ = self.create_subscription(Image, 'camera_topic', self.image_callback, 10)
        self.position_pub_ = self.create_publisher(String, 'object_position', 10)
        self.image_pub_ = self.create_publisher(Image, 'processed_image', 10)
        #self.start_calibration_service_ = self.create_service(Trigger, 'start_calibration', self.start_calibration_callback)
        #self.finish_calibration_service_ = self.create_service(Trigger, 'finish_calibration', self.finish_calibration_callback)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Display image
            #cv2.imshow("camera", frame)
            #cv2.waitKey(1)
            if self.calibration_in_progress_:
                self.calibrate_color(frame)
            else:
                self.process_frame(frame)
            processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub_.publish(processed_msg)
        except CvBridgeError as e:
            self.get_logger().error("Could not convert from '%s' to 'bgr8'." % msg.encoding)
    '''    
    def calibrateColor(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Calculate the size of the ROI
        roiWidth = frame.shape[1] / 25  # 1/25th of the frame's width
        roiHeight = frame.shape[0] / 20 # 1/20th of the frame's height

        # Calculate the starting point (top-left corner) of the ROI
        roiX = (frame.shape[1] - roiWidth) / 2
        roiY = (frame.shape[0] - roiHeight) / 2

        # Create the ROI centered in the frame
        roi = (roiX, roiY, roiWidth, roiHeight)
        cv2.rectangle(frame, (roiX, roiY), (roiX+roiWidth, roiY+roiHeight), (0, 0, 255), 2) # Optional: Draw the ROI for visualization
        hsvRoi = hsv[roiY:roiY+roiHeight, roiX:roiX+roiWidth]

        self.target_hsv_min_ = self.computeMedianHSV(hsvRoi, -35)
        self.target_hsv_max_ = self.computeMedianHSV(hsvRoi, 35)

        self.get_logger().info("Color calibration completed. New HSV Range: [%f, %f, %f] - [%f, %f, %f]",
                    self.target_hsv_min_[0], self.target_hsv_min_[1], self.target_hsv_min_[2],
                    self.target_hsv_max_[0], self.target_hsv_max_[1], self.target_hsv_max_[2])
    '''            
    def process_frame(self, frame):
        
        # Convert the image from BGR to HSV color space
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        '''
        # Define range for red color in HSV
        lower_red = np.array([0, 140, 140])
        upper_red = np.array([10, 255, 175])
        '''
        # Lower range of red
        lower_red_1 = np.array([0, 50, 50])
        upper_red_1 = np.array([10, 255, 255])

        # Upper range of red
        lower_red_2 = np.array([170, 50, 50])
        upper_red_2 = np.array([180, 255, 255])
        # Create masks for red color
        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)

        # Combine the masks
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply the mask to the image
        red_result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

        # Threshold the HSV image to get only red colors

        #mask = cv2.inRange(hsv_image, lower_red, upper_red)
        def is_circle_too_close(circle_centers, center, min_distance):
                for existing_center in circle_centers:
                    distance = ((center[0] - existing_center[0])**2 + (center[1] - existing_center[1])**2)**0.5
                    if distance < min_distance:
                        return True
                return False
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circle_centers = []  # List to store centers of already drawn circles
        # Draw a circle around the red object
        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Calculate the center and radius of the minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            # Draw the circle only if the radius is greater than a minimum threshold
            min_distance_threshold = 20  # Adjust this threshold as needed
            min_area_threshold = 1000     # Adjust this threshold as needed
            min_radius_threshold = 1000   # Adjust this threshold as needed
            if radius > min_radius_threshold and area > min_area_threshold:
                radius = int(radius)
            if is_circle_too_close(circle_centers, center, min_distance_threshold):
                continue
            # Draw the circle in the original image
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            circle_centers.append(center)
            
    '''    
    def process_frame(self, frame):
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_image, self.target_hsv_min_, self.target_hsv_max_)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = frame.shape[0] * frame.shape[1] / 1000.0  # Minimum area to consider
        object_centers = []  # Store centers of detected objects

        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > min_area:
                M = cv2.moments(contour)
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                object_centers.append((cX, cY))
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

        # Output the number of detected objects and their positions
        self.get_logger().info(f"Detected {len(object_centers)} objects")
        for i, center in enumerate(object_centers):
            self.get_logger().info(f"Object {i + 1} at {center}")
    
        # Calculate and draw lines between detected objects
        for i in range(len(object_centers)):
            for j in range(i + 1, len(object_centers)):
                distance = np.linalg.norm(np.array(object_centers[i]) - np.array(object_centers[j]))
                cv2.line(frame, object_centers[i], object_centers[j], (0, 255, 255), 2)
                midpoint = ((object_centers[i][0] + object_centers[j][0]) // 2, (object_centers[i][1] + object_centers[j][1]) // 2)
                cv2.putText(frame, f"{int(distance)} px", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    '''
    '''    
    def start_calibration_callback(self, request, response):
        if not self.calibration_in_progress_:
            self.calibration_in_progress_ = True
            self.get_logger().info("Color calibration started.")
            response.success = True
        else:
            self.get_logger().warn("Color calibration is already in progress.")
            response.success = False
        return response
    
    def finish_calibration_callback(self, request, response):
        if self.calibration_in_progress_:
            self.calibration_in_progress_ = False
            self.get_logger().info("Color calibration finished.")
            response.success = True
        else:
            self.get_logger().warn("No color calibration in progress.")
            response.success = False
        return response
    '''
def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

