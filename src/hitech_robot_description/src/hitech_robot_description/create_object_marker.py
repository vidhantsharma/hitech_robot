from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy

def create_cylinder_marker():
    # Create the marker message
    marker = Marker()
    marker.header.frame_id = "world"  # Ensure the frame matches the world frame in your Gazebo simulation
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cylinder"
    marker.id = 0
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    # Set the position of the cylinder (same pose as in your Gazebo model)
    marker.pose.position.x = 0
    marker.pose.position.y = -2.5
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1.0

    # Set the scale (same as in your Gazebo model)
    marker.scale.x = 0.1  # Diameter of the cylinder
    marker.scale.y = 0.1  # Diameter of the cylinder
    marker.scale.z = 0.3  # Length of the cylinder

    # Set the color (red as per your Gazebo model)
    marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    return marker
