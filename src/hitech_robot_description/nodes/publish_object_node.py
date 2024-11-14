import rospy
from visualization_msgs.msg import Marker
from hitech_robot_description.create_object_marker import create_cylinder_marker

def publish_cylinder_marker():
    rospy.init_node('cylinder_marker_publisher')

    # Create a publisher for visualization marker
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Create the cylinder marker
    marker = create_cylinder_marker()

    # Publish the marker in a loop
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()  # Update timestamp
        marker_pub.publish(marker)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        publish_cylinder_marker()
    except rospy.ROSInterruptException:
        pass
