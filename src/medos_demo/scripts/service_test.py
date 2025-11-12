#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from medos_demo.srv import NextFrontier, NextFrontierResponse, NextFrontierRequest

class ServiceClient:

    def __init__(self):
        rospy.init_node('service_test_node')

        service_name = '/cairo_bebop/next_frontier'
        rospy.wait_for_service(service_name)
        self.request_next_frontier = rospy.ServiceProxy(service_name, NextFrontier)

    def ask_for_frontier(self):
        try:
            request = NextFrontierRequest()
            response = self.request_next_frontier(request)
            if response.frontierExists:
                rospy.loginfo("Received frontier point: x={}, y={}".format(response.frontierPoint.x, response.frontierPoint.y))
            else:
                rospy.loginfo("No frontier available.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
        
    
    def runNode(self):
        self.ask_for_frontier()
        rospy.sleep(1)
        self.ask_for_frontier()
        rospy.sleep(1)
        self.ask_for_frontier()
        rospy.sleep(1)
        self.ask_for_frontier()
        rospy.spin()

if __name__ == '__main__':
    try:
        client_node = ServiceClient()
        client_node.runNode()
    except rospy.ROSInterruptException:
        pass