#!/usr/bin/env python

import rospy

class WaypointController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waypoint_controller_node', anonymous=True)

        rospy.loginfo("Waypoint Contrller Node initialized.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        print("starting node...")
        node = WaypointController()
        node.run()
    except rospy.ROSInterruptException:
        pass
