#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose2D

class DummyPathPlanner:

    def __init__(self):
        rospy.init_node('dummy_planner')

        
        try:
            # Get the input and output topics from parameters (no default values)
            self.paths = rospy.get_param('~go1_paths', "")

        except KeyError as e:
            rospy.logerr("Required parameter {} not provided. Shutting down.".format(str(e)))
            rospy.signal_shutdown("Missing parameter: {}".format(str(e)))
            return
        
        self.goals_sub = rospy.Subscriber('goals', Point, self._on_receive_goal)
        self.waypoint_pub = rospy.Publisher('waypoint', Pose2D, queue_size=10)

        
    def _on_receive_goal(self, point_msg):
        goal_x = point_msg.x
        goal_y = point_msg.y

        best_path_key = None
        nearest_goal_dist = float("inf")
        for path_name, waypoint_list in self.paths.items():
            endpoint = waypoint_list[-1]

            dist = (endpoint['x'] - goal_x) ** 2 + (endpoint['y'] - goal_y) ** 2
            if dist < nearest_goal_dist:
                best_path_key = path_name
                nearest_goal_dist = dist
        
        if best_path_key is not None:
            for waypoint in self.paths[best_path_key]:
                pose_msg = Pose2D()
                pose_msg.x = waypoint['x']
                pose_msg.y = waypoint['y']
                pose_msg.theta = waypoint['theta']

                self.waypoint_pub.publish(pose_msg)

    def runNode(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        map_node = DummyPathPlanner()
        map_node.runNode()
    except rospy.ROSInterruptException:
        pass