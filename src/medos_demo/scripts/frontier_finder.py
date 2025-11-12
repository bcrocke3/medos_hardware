#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from medos_demo.srv import NextFrontier, NextFrontierResponse

class DummyFrontierFinder:

    def __init__(self):
        rospy.init_node('dummy_frontier_finder')

        
        try:
            # Get the input and output topics from parameters (no default values)
            self.frontier_list = rospy.get_param('~frontier_list', "")
            self.frontier_index = 0

        except KeyError as e:
            rospy.logerr("Required parameter {} not provided. Shutting down.".format(str(e)))
            rospy.signal_shutdown("Missing parameter: {}".format(str(e)))
            return
        
        # create service/server here
        self.frontier_server = rospy.Service('next_frontier', NextFrontier, self._on_receive_next_frontier_request)
        
    def _on_receive_next_frontier_request(self, request):
        frontier_response = NextFrontierResponse()

        if len(self.frontier_list) > self.frontier_index:
            frontier_point = self.frontier_list[self.frontier_index]
            frontier_response.frontierExists = True
            frontier_response.frontierPoint = Point(frontier_point['x'], frontier_point['y'], 0.0)
            self.frontier_index += 1
        else:
            frontier_response.frontierExists = False
            frontier_response.frontierPoint = Point()

            rospy.logwarn("No frontiers available to provide.")
        
        return frontier_response
    
    def runNode(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        frontier_finder_node = DummyFrontierFinder()
        frontier_finder_node.runNode()
    except rospy.ROSInterruptException:
        pass