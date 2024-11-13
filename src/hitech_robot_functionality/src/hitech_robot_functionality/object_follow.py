import random
from geometry_msgs.msg import Twist
# from object_detection import objectDetection
from .object_detection import objectDetection
import rospy


class objectFollow:
    def __init__(self):
        # Initialize Object Detection
        self.object_detector = objectDetection()

        # Parameters for movement
        self.forward_speed = 0.1
        self.turn_speed = 0.2
        self.random_distance = 0.5  # Distance to move randomly when object is not found
        self._kp = 0.2

        # Initialize object position
        self.object_position = self.object_detector.relative_position

        rospy.loginfo("Object Follower initialized")

    def move_towards_object(self, cmd_pub):
        """Move the robot towards the object."""
        self.object_position = self.object_detector.relative_position
        rospy.loginfo(f"relative position: {self.object_position}")
        twist = Twist()

        if self.object_position is not None:  # Object detected
            if(abs(self.object_position[0].item()) < 0.55): # depth < 0.55m
                self.stop_robot(cmd_pub)
            object_position_y = self.object_position[1].item()
            
            rospy.loginfo(f"object position is : {object_position_y}")

            twist.linear.x = -self.forward_speed
            twist.angular.z = self._kp * object_position_y

            rospy.loginfo(f"Moving towards object at {self.object_position}")
            cmd_pub.publish(twist)
        else:
            # If no object is detected, rotate 360 degrees, then move 0.5 meters in random direction
            self.search_for_object(cmd_pub)

    def search_for_object(self, cmd_pub):
            """Search for the object by rotating 360 degrees and moving randomly if not found."""
            self.object_position = self.object_detector.relative_position
            twist = Twist()

            # Start rotating (turn 360 degrees at a constant speed)
            twist.angular.z = self.turn_speed  # Rotate at a constant speed
            # cmd_pub.publish(twist)

            # Rotate for 360 degrees (check if the object is detected)
            rotation_duration = 2 * 3.1416 / abs(self.turn_speed)  # Time to complete 360 degrees rotation
            rospy.loginfo(f"Rotating for {rotation_duration:.2f} seconds")
            
            start_time = rospy.Time.now()
            rate = rospy.Rate(10)
            
            while not rospy.is_shutdown():
                self.object_position = self.object_detector.relative_position
                # Check for object detection while rotating
                if self.object_position is not None:
                    rospy.loginfo("Object found during rotation!")
                    break  # Stop rotating if object is found
                
                elapsed_time = rospy.Time.now() - start_time
                if elapsed_time.to_sec() > rotation_duration:
                    rospy.loginfo("Completed 360-degree rotation without detecting object")
                    break  # Stop rotating after completing a full rotation
                
                cmd_pub.publish(twist)
                
                rate.sleep()

            # Stop rotation after completing 360-degree rotation or finding the object
            twist.angular.z = 0
            cmd_pub.publish(twist)

            # If the object is still not detected, move 0.5 meters in a random direction
            if self.object_position is None:
                rospy.loginfo("Object not found, moving randomly for 0.5 meters")
                self.move_randomly(cmd_pub)

    def move_randomly(self, cmd_pub):
        """Move the robot in a random direction for 0.5 meters."""
        self.object_position = self.object_detector.relative_position
        twist = Twist()

        # Move 0.5 meters in a random direction
        random_direction = random.choice([1, -1])  # 1 or -1 for random direction
        twist.linear.x = random_direction * self.forward_speed  # Move forward in the chosen direction
        # cmd_pub.publish(twist)

        # Periodically check if the object is detected during the movement
        rate = rospy.Rate(10)  # Check 10 times per second
        distance_moved = 0
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            self.object_position = self.object_detector.relative_position
            if self.object_position is not None:
                rospy.loginfo("Object found during random movement!")
                break  # Stop moving if object is found

            current_time = rospy.Time.now()
            distance_moved = self.forward_speed * (current_time - start_time).to_sec()

            if distance_moved >= self.random_distance:
                rospy.loginfo("Moved 0.5 m without object detection")
                break  # Stop moving if the robot has moved 0.5 meters
            
            cmd_pub.publish(twist)

            rate.sleep()

        # Stop movement
        twist.linear.x = 0
        cmd_pub.publish(twist)

        # After movement, rotate again if no object is found
        if self.object_position is None:
            self.search_for_object(cmd_pub)
            
    def stop_robot(self, cmd_pub):
        """Stop the robot and shut down the node."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        cmd_pub.publish(twist)  # Send stop command
        rospy.loginfo("Robot stopped.")

        # Shut down the node gracefully
        rospy.signal_shutdown("Object reached or no further action required.")
        rospy.loginfo("Node shutdown initiated.")
        
        
        
