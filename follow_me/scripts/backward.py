#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def move_robot():
    rospy.init_node('linear_movement_node', anonymous=True)
    
    # Publisher for velocity commands
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    
    # Publisher for node completion status
    node_complete_publisher = rospy.Publisher('/node_complete', Bool, queue_size=10)
    
    # Set the movement duration
    move_duration = 5.0  # seconds
    
    # Set the rate of the loop
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message with the desired linear speed
    velocity_msg = Twist()
    velocity_msg.linear.x = -0.2  # Move backward
    velocity_msg.angular.z = 0.0  # No rotation

    # Record the start time
    start_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        
        # Check if the duration has passed
        if current_time - start_time < move_duration:
            # Publish the velocity command
            velocity_publisher.publish(velocity_msg)
        else:
            # Stop the robot
            velocity_msg.linear.x = 0.0
            velocity_publisher.publish(velocity_msg)
            
            # Publish completion status
            node_complete_publisher.publish(True)
            
            # Shutdown the node
            rospy.signal_shutdown("Movement complete")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
