#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
import math


class Go1PoseFilter:

    def __init__(self):
        rospy.init_node('go1_pose_filter', anonymous=True)

        _ = PoseStamped()



        rospy.wait_for_message('/vrpn_client_node/go1/pose', PoseStamped)
        self.full_pose_subscriber = rospy.Subscriber('/vrpn_client_node/go1/pose', PoseStamped, self._onReceiveFullPose)
        self.pose_2d_publisher = rospy.Publisher('/cairo_go1/pose', Pose2D, queue_size=20)

        rospy.loginfo("Node initialized.")

    def _quaternion_to_theta(self, x, y, z, w):
        # Yaw (rotation around Z axis)
        theta = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return theta  # in radians

    def _onReceiveFullPose(self, pose_stamped):
        # rospy.loginfo("pose msg received.")
        header = pose_stamped.header

        pose = pose_stamped.pose
        quat = pose.orientation

        # header_string = "Header: " + str(header.seq) + " " + header.frame_id

        # rospy.loginfo(header_string)
        # rospy.loginfo("Position: ", pose.position.x, pose.position.y, pose.position.z)

        pose_2d = Pose2D()
        pose_2d.x = pose.position.x
        pose_2d.y = pose.position.y
        pose_2d.theta = self._quaternion_to_theta(quat.x, quat.y, quat.z, quat.w)

        self.pose_2d_publisher.publish(pose_2d)

    def runNode(self):
        rospy.spin()


if __name__ == '__main__':
    node = Go1PoseFilter()
    node.runNode()