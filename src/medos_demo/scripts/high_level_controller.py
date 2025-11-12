#!/usr/bin/env python
import rospy
from enum import Enum
from geometry_msgs.msg import Twist, Pose2D, Point
from std_msgs.msg import Empty, Int32
import math
from medos_demo.msg import RecapCommand, RecapTelemetry, Target
from medos_demo.srv import NextFrontier

# Collect all telemetry
# position for bebop, go1
# targets detected
# done_exploring
# hasfoundfrontiers
# done_with_science
# system_status
# publishes to telemetry topic

# subscribes to recap command topic
# translates out 

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
        
        self.namespace = ""
        if self.agent_type == AgentType.SCOUT:
            self.namespace = 'cairo_bebop/'
        elif self.agent_type == AgentType.SCIENTIST:
            self.namespace = 'cairo_go1/'

        # ROS pub/sub
        rospy.Subscriber('command', RecapCommand, self.onRecapCommandReceived)
        rospy.Subscriber('pose', Pose2D, self.onPoseReceived)
        rospy.Subscriber('done_with_science', Empty, self.onDoneWithScienceReceived)
        rospy.Subscriber('detected_target', Target, self.onDetectedTargetReceived)
        # rospy.Subscriber('frontier_goals', Point, self.onFrontierGoalsReceived)
        self.telemetry_publisher = rospy.Publisher('telemetry', RecapTelemetry, queue_size=20)
        self.go_wait_publisher = rospy.Publisher('go_wait', Int32, queue_size=10)
        self.goals_publisher = rospy.Publisher('goals', Point, queue_size=10)
        # self.explore_publisher = rospy.Publisher('explore', Empty, queue_size=10)

        self.next_frontier_service = rospy.ServiceProxy('next_frontier', NextFrontier)
        rospy.Timer(rospy.Duration(0.1), self.publishTelemetry)

        rospy.loginfo("Node initialized. Agent type: " + str(self.agent_type))

        # States
        self.position = Pose2D()
        self.detected_targets = []
        # self.done_exploring = False
        # self.has_found_frontiers = False
        self.done_with_science = False
        self.reached_navigation_target = False
        self.system_status = "OK"

        self.hasFoundAllFrontiers = False
        self.hasReachedCurrentFrontier = False

        self.current_frontier_goal = None
        self.current_navigation_goal = None
        self.current_science_target = None
    
    def publishTelemetry(self, event):
        telemetry_msg = RecapTelemetry()
        telemetry_msg.position = self.position
        telemetry_msg.detectedTargets = self.detected_targets
        telemetry_msg.hasFoundAllFrontiers = self.hasFoundAllFrontiers
        telemetry_msg.hasReachedCurrentFrontier = self.hasReachedCurrentFrontier
        telemetry_msg.hasReachedNavigationTarget = self.reached_navigation_target
        telemetry_msg.hasFinishedAnalyzingTarget = self.done_with_science
        telemetry_msg.systemStatus = self.system_status

        self.telemetry_publisher.publish(telemetry_msg)

    def onRecapCommandReceived(self, msg):
        if msg.behavior == RecapCommand.NOCHANGE:
            print("No change")
        elif msg.behavior == RecapCommand.IDLE:
            print("Idle")
            self.publish_wait()
        elif msg.behavior == RecapCommand.EXPLORE:
            print("Explore")
            self.hasFoundAllFrontiers = False
            self.handleNextFrontier()
            self.publish_go()
        elif msg.behavior == RecapCommand.ANALYZE_TARGET:
            print("Analyze Target")
            self.done_with_science = False
            self.current_science_target = msg.target.location
            self.publish_wait()
            self.waitForScience()
        elif msg.behavior == RecapCommand.GOTO_TARGET:
            print("Go to Target:", msg.target)
            self.current_navigation_goal = msg.target.location
            self.reached_navigation_target = None
            self.goals_publisher.publish(msg.target.location)
            self.publish_go()
            # ask for frontier
            # upon receive frontier, publish goal


        # message type: RecapCommand
        # print("received recap command")
    
    def waitForScience(self):
        rospy.Timer(rospy.Duration(5.0), self.returnScienceDone, oneshot=True)
    
    def returnScienceDone(self, event):
        self.done_with_science = True

    def publish_go(self):
        self.go_wait_publisher.publish(Int32(1))
    
    def publish_wait(self):
        self.go_wait_publisher.publish(Int32(0))

    def getNextFrontier(self):
        self.hasReachedCurrentFrontier = False
        try:
            response = self.next_frontier_service()
            if response.frontierExists:
                return response.frontierPoint
            else:
                self.hasFoundAllFrontiers = True
                self.hasReachedCurrentFrontier = True
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None
    
    def handleNextFrontier(self):
        frontier = self.getNextFrontier()
        if frontier is not None:
            self.current_frontier_goal = frontier
            self.goals_publisher.publish(self.current_frontier_goal)
    
    def check_if_reached_goal(self, goal, current_position):
        if goal is None:
            return True
        distance = math.sqrt((goal.x - current_position.x) ** 2 +
                             (goal.y - current_position.y) ** 2)
        threshold = 0.15  # meters
        return distance <= threshold
    
    def onPoseReceived(self, msg):
        self.position = msg
        # Note: only checking navigation goals for scientist; as implemented, scout never has navigation goals
        if self.agent_type == AgentType.SCIENTIST:
            goal_reached = self.check_if_reached_goal(self.current_navigation_goal, self.position)
            if goal_reached:
                self.reached_navigation_target = True
        elif self.agent_type == AgentType.SCOUT:
            goal_reached = self.check_if_reached_goal(self.current_frontier_goal, self.position)
            if goal_reached:
                self.hasReachedCurrentFrontier = True
            # if goal_reached:
            #     self.handleNextFrontier()
        

    def onDoneWithScienceReceived(self, msg):
        self.done_with_science = True

    def onDetectedTargetReceived(self, msg):
        self.detected_targets.append(msg)

    def run_node(self):
        rospy.spin()

   
if __name__ == '__main__':
    try:
        controller_node = HighLevelController()
        controller_node.run_node()

    except rospy.ROSInterruptException:
        pass
