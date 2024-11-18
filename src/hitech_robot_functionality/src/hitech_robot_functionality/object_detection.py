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

        # Load YOLO model
        self.net = cv2.dnn.readNetFromDarknet("src/hitech_robot_functionality/config/yolov3-tiny.cfg", "src/hitech_robot_functionality/config/yolov3-tiny.weights")
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Load COCO class labels
        with open("src/hitech_robot_functionality/data/coco.names", "r") as f:
            self.classes = f.read().strip().split("\n")

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

        # Prepare the image for YOLO
        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        
        # Perform forward pass to get detections
        detections = self.net.forward(self.output_layers)

        for detection in detections:
            for obj in detection:
                scores = obj[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                # Filter detections by confidence threshold
                if confidence > 0.5:
                    # Get bounding box coordinates
                    center_x = int(obj[0] * width)
                    center_y = int(obj[1] * height)
                    w = int(obj[2] * width)
                    h = int(obj[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    # Draw bounding box and label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = f"{self.classes[class_id]}: {confidence:.2f}"
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Get depth information
                    depth = self.get_depth(center_x, center_y)

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
