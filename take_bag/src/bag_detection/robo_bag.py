#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class TurtleBotApproach:
    def __init__(self):
        rospy.init_node('turtlebot_proportional_control_node')
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)
        rospy.Subscriber('/bag_distance', Float32, self.distance_callback)
        rospy.Subscriber('/bag_angle', Float32, self.angle_callback)

        self.target_distance = 0.21  # Target distance in meters
        self.kp_dist = 0.4  # Proportional gain for distance
        self.kp_ang = 4.0  # Proportional gain for angle
        self.kd_dist = 0.4  # Derivative gain for distance
        self.kd_ang = 3.5  # Derivative gain for angle
        self.ki_dist = 0.1  # Integral gain for distance
        self.ki_ang = 0.1  # Integral gain for angle

        self.max_linear_vel = 0.1
        self.max_angular_vel = 0.3

        self.cmd_vel = Twist()
        self.distance_to_bag = 0
        self.angle_to_bag = 0
        self.previous_error_dist = 0
        self.previous_error_ang = 0
        self.integral_dist = 0
        self.integral_ang = 0
        self.has_reached_distance = False

    def distance_callback(self, msg):
        # Convert distance from millimeters to meters
        self.distance_to_bag = msg.data / 10000.0  # Conversion factor for mm to m
        self.update_velocity()

    def angle_callback(self, msg):
        self.angle_to_bag = msg.data
        self.update_velocity()

    def update_velocity(self):
        if self.distance_to_bag is None or self.angle_to_bag is None:
            rospy.loginfo("Waiting for distance and angle measurements...")
            return

        error_dist = self.distance_to_bag - self.target_distance
        error_ang = self.angle_to_bag

        derivative_dist = error_dist - self.previous_error_dist
        derivative_ang = error_ang - self.previous_error_ang

        self.integral_dist += error_dist
        self.integral_ang += error_ang

        self.previous_error_dist = error_dist
        self.previous_error_ang = error_ang

        control_output_dist = (self.kp_dist * error_dist + self.kd_dist * derivative_dist + self.ki_dist * self.integral_dist)
        control_output_ang = (self.kp_ang * error_ang + self.kd_ang * derivative_ang + self.ki_ang * self.integral_ang)

        if abs(error_dist) <= 0.03:
            if not self.has_reached_distance:
                rospy.loginfo("Target distance reached. Stopping robot.")
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                self.vel_pub.publish(self.cmd_vel)
                self.reach_distance_pub.publish(True)
                self.has_reached_distance = True
            return

        self.cmd_vel.angular.z = max(min(-control_output_ang, self.max_angular_vel), -self.max_angular_vel)
        self.cmd_vel.linear.x = max(min(control_output_dist, self.max_linear_vel), -self.max_linear_vel)

        self.vel_pub.publish(self.cmd_vel)
        rospy.loginfo(f'Velocity command - Linear: {self.cmd_vel.linear.x:.2f} m/s, Angular: {self.cmd_vel.angular.z:.2f} rad/s')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        turtlebot_approach = TurtleBotApproach()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass