#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class ObstacleAvoidanceController:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_controller', anonymous=True)

        # Publishers
        self.return_to_direction_pub = rospy.Publisher('/return_to_direction_flag', Bool, queue_size=10)

        # Subscribers
        self.obstacle_avoided_sub = rospy.Subscriber('/obstacle_avoided', Bool, self.obstacle_avoided_callback)

        # Internal state
        self.is_returning = False  # To track if the return command has been sent

    def obstacle_avoided_callback(self, msg):
        if msg.data and not self.is_returning:
            rospy.loginfo("Obstacle avoided, triggering return to original direction.")
            self.return_to_direction_pub.publish(Bool(data=True))
            self.is_returning = True  # Set the state to indicate the command has been sent

            # Optional: reset the flag after a delay
            rospy.sleep(2)  # Adjust the delay as needed
            self.return_to_direction_pub.publish(Bool(data=False))

            rospy.loginfo("Return to original direction command sent.")

    def reset_return_flag(self):
        self.is_returning = False  # Allow the flag to be set again after the obstacle is avoided

if __name__ == '__main__':
    try:
        controller = ObstacleAvoidanceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
