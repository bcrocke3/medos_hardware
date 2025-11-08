#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Empty
import math
import datetime 
from collections import deque

class BebopWaypointController:
    def __init__(self):
        rospy.init_node('bebop_waypoint_controller')

        self.last_pose_time = None  # Tracks last valid pose message
        self.pose_timeout = rospy.Duration(1.0)     # seconds

        self.pose = Pose2D()
        self.waypoints = deque()

        # Parameters
        self.distance_tolerance = 0.15  # meters
        self.angle_tolerance = 0.3     # radians

        self.max_linear_speed = 0.05     # m/s
        self.max_angular_speed = 0.25   # rad/s

        # Control gains
        self.kp_x = 0.1
        self.kp_y = 0.1
        self.kp_theta = 0.3

        self.stopped = False

        # ROS pub/sub
        rospy.Subscriber('/cairo_bebop/stop', Empty, self.stop_callback)
        rospy.Subscriber('/cairo_bebop/pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/cairo_bebop/waypoint', Pose2D, self.waypoint_callback)
        self.cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        self.control_loop()

    def stop_callback(self, msg):
        self.stopped = True
        self.waypoints.clear()

    def pose_callback(self, msg):
        self.pose = msg
        self.last_pose_time = rospy.Time.now()

    def waypoint_callback(self, msg):
        rospy.loginfo("Received new waypoint: (%.2f, %.2f, %.2f)", msg.x, msg.y, msg.theta)
        self.waypoints.append(msg)

    def control_loop(self):
        rate = rospy.Rate(20)  # 20 Hz control loop
        while not rospy.is_shutdown():

            if self.stopped:
                self.waypoints.clear()

            if self.last_pose_time is not None and rospy.Time.now() - self.last_pose_time > self.pose_timeout:
                self.waypoints.clear()
                rospy.logwarn("Last pose too stale. Cleared waypoints.")

            if self.waypoints:

                # compute command
                target = self.waypoints[0]
                cmd = self.compute_command(target)
                self.cmd_pub.publish(cmd)

                if self.goal_reached(target):
                    rospy.loginfo("Reached waypoint: (%.2f, %.2f, %.2f)", target.x, target.y, target.theta)
                    self.waypoints.popleft()
            else:
                self.cmd_pub.publish(Twist())  # No command (robot stays still)

            rate.sleep()

    def compute_command(self, target):
        cmd = Twist()

        theta = self.pose.theta

        # Compute position error
        dx = target.x - self.pose.x
        dy = target.y - self.pose.y
        dtheta = self.normalize_angle(target.theta - self.pose.theta)
        rospy.loginfo("error: dx, dy, dtheta: " + str(dx) + ", " + str(dy) + ", " + str(dtheta))

        # Robot frame error
        error_x =  dx * math.cos(theta) + dy * math.sin(theta)
        error_y = -dx * math.sin(theta) + dy * math.cos(theta)

        # Apply proportional control
        cmd.linear.x = self.kp_x * error_x
        cmd.linear.y = self.kp_y * error_y
        cmd.angular.z = self.kp_theta * dtheta

        # Clamp to max speeds
        cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.x))
        cmd.linear.y = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.y))
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        return cmd

    def goal_reached(self, target):
        dx = target.x - self.pose.x
        dy = target.y - self.pose.y
        dtheta = self.normalize_angle(target.theta - self.pose.theta)

        distance_error = math.hypot(dx, dy)

        # return distance_error < self.distance_tolerance
        return (distance_error < self.distance_tolerance and
                abs(dtheta) < self.angle_tolerance)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle



if __name__ == '__main__':
    try:
        BebopWaypointController()
    except rospy.ROSInterruptException:
        pass
