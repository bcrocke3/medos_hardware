#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header, String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import os

class RosMap:

    def __init__(self):
        rospy.init_node('ros_map')

        self.map_publisher = rospy.Publisher("/full_map", OccupancyGrid, queue_size=20)
        # self.test_pub = rospy.Publisher("/test_topic", String, queue_size=20)


        map_path = rospy.get_param('~map_file_path', "")
        if os.path.exists(map_path):
            self._load_map_from_file(map_path)
        else:
            rospy.logerr("Map File Path does not exist.")


        rospy.Timer(rospy.Duration(0.5), self._publishMapMessage) # pub map every 10 seconds


    def _load_map_from_file(self, file_path):
        data = np.load(file_path)

        # flip over y-axis
        self.map_grid = np.flip(100 * data, axis=1)

        self.map_resolution = 0.1 # meters / cell
        self.map_origin = Pose()
        self.map_origin.position.x = -2.0
        self.map_origin.position.y = -1.5
        self.map_origin.position.z = 0.0
        self.map_origin.orientation.x = 0.0
        self.map_origin.orientation.y = 0.0
        self.map_origin.orientation.z = 0.0
        self.map_origin.orientation.w = 1.0


    def _publishMapMessage(self, _):
        map_meta_data_msg = MapMetaData()
        map_meta_data_msg.resolution = self.map_resolution
        map_meta_data_msg.width = self.map_grid.shape[1]
        map_meta_data_msg.height = self.map_grid.shape[0]
        map_meta_data_msg.origin = self.map_origin

        map_msg = OccupancyGrid()
        map_msg.data = self.map_grid.flatten()
        map_msg.info = map_meta_data_msg
        map_msg.header = Header()

        self.map_publisher.publish(map_msg)


    def _onCollisionCheckRequest(self, collision_check_request):
        pass

    def runNode(self):
        rospy.spin()



if __name__ == '__main__':
    try:
        map_node = RosMap()
        map_node.runNode()
    except rospy.ROSInterruptException:
        pass