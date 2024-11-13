#!/usr/bin/env python3
import rospy
from hitech_robot_functionality.object_detection import objectDetection  # Corrected import
from sensor_msgs.msg import Image, CameraInfo

if __name__ == '__main__':
    rospy.init_node('object_detection_node', anonymous=True)
    # Initialize the object detection node
    object_detection_node = objectDetection()

    # Subscribe to the camera info and image topics
    rospy.Subscriber('/camera_link/color/camera_info', CameraInfo, object_detection_node.camera_info_callback)
    rospy.Subscriber('/camera_link/color/image_raw', Image, object_detection_node.image_callback)

    # Start the ROS event loop
    rospy.spin()
