#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
import math


class PoseFilter:

    def __init__(self):
        rospy.init_node('pose_filter', anonymous=True)

        try:
            # Get the input and output topics from parameters (no default values)
            input_topic = rospy.get_param('~input_topic', "")
            output_topic = rospy.get_param('~output_topic', "")

        except KeyError as e:
            rospy.logerr("Required parameter {} not provided. Shutting down.".format(str(e)))
            rospy.signal_shutdown("Missing parameter: {}".format(str(e)))
            return

        rospy.wait_for_message(input_topic, PoseStamped)
        self.full_pose_subscriber = rospy.Subscriber(input_topic, PoseStamped, self._onReceiveFullPose)
        self.pose_2d_publisher = rospy.Publisher(output_topic, Pose2D, queue_size=20)

        rospy.loginfo("Pose Filter Node initialized, subscribing to {}, publishing to {}".format(input_topic, output_topic))

    def _quaternion_to_theta(self, x, y, z, w):
        # Yaw (rotation around Z axis)
        theta = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return theta  # in radians

    def _onReceiveFullPose(self, pose_stamped):
        # rospy.loginfo("pose msg received.")
        header = pose_stamped.header

        pose = pose_stamped.pose
        quat = pose.orientation

        pose_2d = Pose2D()
        pose_2d.x = pose.position.x
        pose_2d.y = pose.position.y
        pose_2d.theta = self._quaternion_to_theta(quat.x, quat.y, quat.z, quat.w)

        self.pose_2d_publisher.publish(pose_2d)

    def runNode(self):
        rospy.spin()


if __name__ == '__main__':
    node = PoseFilter()
    node.runNode()