#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import tf

class MoveAndTurn:
    def __init__(self):
        rospy.init_node('move_and_turn', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.obstacle_subscriber = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        self.kinect_subscriber = rospy.Subscriber('/obstacle_detected', Bool)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_pose = None
        self.obstacle_detected = False

    def odom_callback(self, data):
        # Update the current pose
        self.current_pose = data.pose.pose

    def obstacle_callback(self, msg):
        # Update the obstacle_detected status
        self.obstacle_detected = msg.data

    def move_straight(self, speed, duration):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.velocity_publisher.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot
        move_cmd.linear.x = 0
        self.velocity_publisher.publish(move_cmd)

    def rotate(self, angular_speed, relative_angle):
        print("Start rotating")
        rotate_cmd = Twist()
        rotate_cmd.linear.x = 0
        rotate_cmd.angular.z = angular_speed

        start_yaw = self.get_yaw()
        relative_angle = abs(relative_angle)
        print(f"the target angle is {relative_angle}")
        
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw()
            acc_yaw = current_yaw - start_yaw
            print(f"the current angle is {acc_yaw}")
            if abs(acc_yaw) > relative_angle:
                break
            self.velocity_publisher.publish(rotate_cmd)
            self.rate.sleep()

        # Stop the robot
        rotate_cmd.angular.z = 0
        self.velocity_publisher.publish(rotate_cmd)

    def get_yaw(self):
        if self.current_pose is None:
            return 0

        orientation = self.current_pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        return yaw

    def run(self):
        rospy.sleep(1)  # Give time for initialization

        # Wait until obstacle is detected
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected, starting the process...")

                # Step 2: Turn 90 degrees to the left with 0.6 rad/s angular speed
                self.rotate(angular_speed=0.6, relative_angle=math.radians(90))

                # Step 3: Move straight for 1 second
                self.move_straight(speed=0.2, duration=2.0)

                # Step 4: Turn 90 degrees to the right with 0.6 rad/s angular speed
                self.rotate(angular_speed=-0.6, relative_angle=-math.radians(90))

                # Step 5: Move straight for another 3 seconds
                self.move_straight(speed=0.2, duration=4.0)

                # Step 6: Turn 90 degrees to the right with 0.6 rad/s angular speed
                self.rotate(angular_speed=-0.6, relative_angle=-math.radians(90))

                # Step 7: Move straight for another 1 second
                self.move_straight(speed=0.2, duration=2.0)

                # Final rotate
                self.rotate(angular_speed=0.6, relative_angle=math.radians(90))

                rospy.loginfo("Task completed!")
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        mover = MoveAndTurn()
        mover.run()
    except rospy.ROSInterruptException:
        pass
