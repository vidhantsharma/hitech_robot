import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rospy
import tf
from geometry_msgs.msg import PointStamped

class objectDetection(object):
    def __init__(self):
        # Initialize the bridge to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        # Camera calibration parameters
        self.camera_info = None

        # Initialize TF listener to get transformations
        self.listener = tf.TransformListener()
        
        self.relative_position = None

    def camera_info_callback(self, msg):
        """ Callback to store camera info """
        self.camera_info = msg

    def image_callback(self, msg):
        """ Callback to process incoming image messages """
        if self.camera_info is None:
            rospy.logwarn("Camera info not received yet")
            return

        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV space
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        # Threshold the image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours of the detected objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Ignore small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw a circle at the center of the object
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

                # Get depth information (if camera is a depth camera)
                depth = self.get_depth(center_x, center_y)

                # If depth information is available
                if depth is not None:
                    # Convert pixel coordinates to camera frame
                    self.relative_position = self.pixel_to_camera_frame(center_x, center_y, depth)
                    
                    rospy.loginfo(f"Relative position: {self.relative_position}")

                    # Get the relative position in world frame using tf
                    object_position_in_world = self.get_position_in_world(self.relative_position)

                    # Display object position in world frame
                    rospy.loginfo(f"Object position in world frame: {object_position_in_world}")
                else:
                    self.relative_position = None

        # Display the result
        # cv2.imshow("Detected Objects", frame)
        # cv2.waitKey(1)

    def get_depth(self, x, y):
        """ Get depth from the depth image at (x, y) coordinates """
        depth_image = rospy.wait_for_message("/camera_link/depth/image_raw", Image)
        depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        return depth_frame[y, x] if depth_image else None

    def pixel_to_camera_frame(self, x, y, depth):
        """ Convert pixel coordinates to camera frame (assuming no distortion) """
        # Use camera_info to get intrinsic parameters
        fx = self.camera_info.K[0]  # Focal length in x
        fy = self.camera_info.K[4]  # Focal length in y
        cx = self.camera_info.K[2]  # Optical center in x
        cy = self.camera_info.K[5]  # Optical center in y

        # Convert pixel coordinates to camera frame
        z_camera = (cy - y) * depth / fy
        x_camera = -depth
        y_camera = (x - cx) * depth / fx

        return np.array([x_camera, y_camera, z_camera])

    def get_position_in_world(self, camera_position):
        """ Transform the relative position from camera frame to world frame using TF """
        try:
            # Wait for the transform from the camera frame to the world frame
            self.listener.waitForTransform("/world", "/camera_link", rospy.Time(0), rospy.Duration(1.0))
            
            # Create a PointStamped message in the camera frame
            camera_point = PointStamped()
            camera_point.header.frame_id = "/camera_link"
            camera_point.point.x = camera_position[0]
            camera_point.point.y = camera_position[1]
            camera_point.point.z = camera_position[2]
            
            # Transform the point from camera frame to world frame
            world_point = self.listener.transformPoint("/world", camera_point)

            return world_point.point

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logwarn("TF transform error")
            return None
