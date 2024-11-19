import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from hitech_robot_functionality.object_follow import objectFollow


class objectFollowNode:
    def __init__(self):
        rospy.init_node('object_follower')

        # Initialize ObjectFollower
        self.follower = objectFollow()

        # Subscribe to camera and camera info topics
        rospy.Subscriber('/camera_link/color/image_raw', Image, self.follower.object_detector.image_callback)
        rospy.Subscriber('/camera_link/color/camera_info', CameraInfo, self.follower.object_detector.camera_info_callback)

        # Publisher to send velocity commands to the robot
        self.cmd_pub = rospy.Publisher('hitech_robot_controller/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Object Follower Node initialized")

    def run(self):
        """Run the object following logic."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.follower.move_towards_object(self.cmd_pub)
            rate.sleep()


if __name__ == '__main__':
    try:
        node = objectFollowNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
