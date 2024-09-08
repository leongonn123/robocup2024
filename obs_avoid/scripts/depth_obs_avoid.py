#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoidance', anonymous=True)
        
        # Create a subscriber to the /obstacle_detected topic
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        
        # Create a publisher to the /cmd_vel topic for controlling the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        # Set up the Twist message for controlling the robot's movement
        self.twist = Twist()

        rospy.loginfo("Obstacle avoidance node started.")
    
    def avoid_obstacle(self):
        # Turn 45 degrees with a small linear speed
        rospy.loginfo("Obstacle detected! Turning 45 degrees.")
        
        # Define the angular speed and the target angle
        angular_speed = 0.3  # radians per second
        linear_speed = 0.05  # meters per second
        target_angle = 0.785  # 45 degrees in radians

        # Initialize rotation parameters
        current_angle = 0.0
        rate = rospy.Rate(10)  # 10 Hz update rate
        
        # Turn the robot
        while current_angle < target_angle and not rospy.is_shutdown():
            self.twist.linear.x = linear_speed
            self.twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
            current_angle += angular_speed / 10.0  # Update the current angle

        # Stop the robot after the turn is completed
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Turn completed. Robot stopped.")
    
    def resume_movement(self):
        # Resume forward movement if no obstacle is detected
        self.twist.linear.x = 0.1  # Adjust speed as needed
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("No obstacle detected. Moving forward.")
    
    def obstacle_callback(self, msg):
        if msg.data:
            self.avoid_obstacle()
        else:
            self.resume_movement()

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass

