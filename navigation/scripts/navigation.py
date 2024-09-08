#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool  # Import to publish boolean messages
from tf.transformations import quaternion_from_euler
import subprocess

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Initialize state variables
        self.start = 1
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")

        # Publisher to indicate completion of the navigation task
        self.complete_pub = rospy.Publisher('/node_complete', Bool, queue_size=1)

        rospy.loginfo("Ready to go")
        rospy.sleep(1)
    
        # Define the target location (point A)
        A_x = 0.466
        A_y = 6.77
        A_theta = 0
        
        quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        self.target_location = Pose(Point(A_x, A_y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        
        self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Robot will go to point A
            if self.start == 1:
                rospy.loginfo("Going to point A")
                rospy.sleep(2)
                self.goal.target_pose.pose = self.target_location
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))

                # Check if the goal was reached
                if waiting and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached point A")
                    rospy.sleep(2)
                    self.complete_pub.publish(True)  # Publish True to /node_complete
                    self.kill_ros_node('/navi_point')  # Kill the node
                    rospy.signal_shutdown("Navigation task completed")
                    break  # Exit the loop after task is done

            rospy.Rate(5).sleep()

    def cleanup(self):
        rospy.loginfo("Shutting down navigation....")
        self.move_base.cancel_goal()
        self.complete_pub.publish(False)  # Publish False indicating the navigation was interrupted

    def kill_ros_node(self, navi_point):
        try:
            subprocess.run(['rosnode', 'kill', navi_point])
            rospy.loginfo(f"Successfully killed {navi_point}")
        except Exception as e:
            rospy.logerr(f"Failed to kill node: {navi_point} due to: {e}")

if __name__ == "__main__":
    rospy.init_node('navi_point', anonymous=True)
    try:
        NavToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
