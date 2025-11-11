#!/usr/bin/env python
import rospy
from enum import Enum
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Empty
import math
from medos_demo.msg import RecapCommand, RecapTelemetry

class AgentType(str, Enum):
    SCOUT = 'SCOUT'
    SCIENTIST = 'SCIENTIST'

class HighLevelController:
    def __init__(self):
        rospy.init_node('high_level_controller', anonymous=True)

        agent_type_str = rospy.get_param("~agent_type", "").strip().upper()

        try:
            self.agent_type = AgentType(agent_type_str)
        except ValueError:
            valid_types = ", ".join(a.name for a in AgentType)
            err_str = "Invalid agent type '" + agent_type_str + "'. \nValid options are: " + valid_types
            rospy.logerr(err_str)

        # ROS pub/sub
        rospy.Subscriber('command', RecapCommand, self.onRecapCommandReceived)
        self.telemetry_publisher = rospy.Publisher('telemetry', RecapTelemetry, queue_size=20)

        rospy.loginfo("Node initialized. Agent type: " + str(self.agent_type))


    def onRecapCommandReceived(self, msg):
        if msg.behavior == RecapCommand.NOCHANGE:
            print("No change")
        elif msg.behavior == RecapCommand.IDLE:
            print("Idle")
        elif msg.behavior == RecapCommand.EXPLORE:
            print("Explore")
        # message type: RecapCommand
        # print("received recap command")

    def run_node(self):
        rospy.spin()

   
if __name__ == '__main__':
    try:
        controller_node = HighLevelController()
        controller_node.run_node()

    except rospy.ROSInterruptException:
        pass
