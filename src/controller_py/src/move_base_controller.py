#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import enum
import random
import threading
import time
import datetime
from shapely.geometry import Point, Polygon

class GeneralArea(enum.Enum):
    NOT_DEFINED = -1
    AREA_RIGHT = 0
    AREA_UP = 1
    AREA_LEFT = 2
    AREA_DOWN = 3
    AREA_MIDDLE = 4

class AreaDelimitations():
    AREAS = {
            'AREA_RIGHT': {
                'P1': Point(1.1313250064849854, 2.4731078147888184),
                'P2': Point(1.4579946994781494, 1.964996576309204),
                'P3': Point(1.4172468185424805, 1.2950527667999268),
                'P4': Point(-1.3090224266052246, 1.3626840114593506),
                'P5': Point(-1.305069088935852, 2.0217161178588867),
                'P6': Point(-1.0252100229263306, 2.5042612552642822),
            },
            'AREA_UP': {
                'P1': Point(1.4591257572174072, 1.908069133758545),
                'P2': Point(1.84257173538208, 1.9523131847381592),
                'P3': Point(2.611079692840576, 0.5322036147117615),
                'P4': Point(2.347233772277832, -0.022695258259773254),
                'P5': Point(2.567981243133545, -0.5458083152770996),
                'P6': Point(1.7939517498016357, -1.9409345388412476),
                'P7': Point(1.3767004013061523, -1.9720466136932373),
            },
            'AREA_LEFT' : {
                'P1': Point(1.3434334993362427, -2.012788772583008),
                'P2': Point(1.3762962818145752, -1.2581532001495361),
                'P3': Point(-1.3986012935638428, -1.2454938888549805),
                'P4': Point(-1.398937702178955, -1.8853110074996948),
                'P5': Point(-1.0495669841766357, -2.4744865894317627),
                'P6': Point(1.098577857017517, -2.4462695121765137),
            },
            'AREA_DOWN': {
                'P1': Point(-1.3720011711120605, -1.88504159450531),
                'P2': Point(-1.7487800121307373, -1.9224839210510254),
                'P3': Point(-2.832045555114746, -0.007140785455703735),
                'P4': Point(-1.6800928115844727, 1.9844343662261963),
                'P5': Point(-1.3233144283294678, 2.001471996307373),
            },
            'AREA_MIDDLE': {
                'P1': Point(-1.2015597820281982, 1.2753223180770874),
                'P2': Point(-1.2302465438842773, -1.2438101768493652),
                'P3': Point(1.2480765581130981, -1.2324957847595215),
                'P4': Point(1.2504326105117798, 1.2257593870162964),
            }
        }   
    
class InitialPoseEstimate(PoseWithCovarianceStamped):

    def __init__(self):
        super().__init__()
        self.header.frame_id = 'map'
        self.header.stamp = rospy.Time.now()

        self.pose.pose.position.x = -1.9900001287460327
        self.pose.pose.position.y = -0.45999985933303833
        self.pose.pose.position.z = 0.0

        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = -4.139211284837073e-08
        self.pose.pose.orientation.w = 0.9999999999999991
        self.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]


class MoveBaseController(threading.Thread):
    
    def __init__(self) -> None:

        threading.Thread.__init__(self)
        rospy.init_node("python_move_base_controller")

        self.goal_action_client = actionlib.ActionClient('move_base', MoveBaseAction)
        self.robot_odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        self.initial_pose_estimate_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.current_estimated_pose = Odometry()

        self.current_area = GeneralArea.NOT_DEFINED
        self.available_areas = {}
        self.createPolygons()

        self.latest_published_feedback = rospy.Time.now()

        self.activeGoal = False

        while not self.goal_action_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("Successfully initialized controller")

    def createPolygons(self) -> None:

        for area, regions in AreaDelimitations.AREAS.items():
            self.available_areas[area] = Polygon(list(regions.values()))
        
    def confirmServerUp(self) -> bool:

        return self.goal_action_client.wait_for_server(rospy.Duration(5))

    def isInsideArea(self, point : Point, polygon: Polygon) -> bool:

        if polygon.contains(point):
            return True
        else:
            return False

    def determineCurrentArea(self, x: float, y: float) -> GeneralArea:

        for area, polygon in self.available_areas.items():
            if self.isInsideArea(Point(x, y), polygon):
                return GeneralArea[area]
            
        return GeneralArea.NOT_DEFINED

    def odomCallback(self, msg: Odometry) -> None:

        self.current_estimated_pose = msg
        self.current_area = self.determineCurrentArea(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def selectRandomArea(self) -> GeneralArea:

        random_area = None
        start_time = datetime.datetime.now()
        current_time = datetime.datetime.now()

        while random_area is None and (current_time - start_time) < datetime.timedelta(seconds=10):
            random_area = random.choice(list(GeneralArea))
            if (random_area != GeneralArea.NOT_DEFINED and random_area != self.current_area):
                break
            current_time = datetime.datetime.now()

        if random_area is None or random_area == GeneralArea.NOT_DEFINED:
            random_area = GeneralArea.AREA_MIDDLE

        return random_area
    
    def selectRandomCoordinatesFromArea(self, desired_polygon : Polygon) -> tuple:

        xmin, ymin, xmax, ymax = desired_polygon.bounds

        while True:

            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            if Point(x, y).within(desired_polygon):
                break

        return (x, y)
    
    def selectRandomOrientation(self) -> float:

        return float(random.randint(0, 360))

    def generateRandomGoal(self) -> MoveBaseGoal:

        area_to_go : GeneralArea = self.selectRandomArea()
        selected_polygon = self.available_areas[area_to_go.name]
        (x, y) = self.selectRandomCoordinatesFromArea(selected_polygon)
        z = self.selectRandomOrientation()

        rospy.loginfo("Goal pose -> Selected area: %s -> x: %f y: %f z: %f", area_to_go.name, x, y, z)

        goal_to_return = MoveBaseGoal()

        goal_to_return.target_pose.header.frame_id = 'map'
        goal_to_return.target_pose.header.stamp = rospy.Time.now()

        goal_to_return.target_pose.pose.position.x = x
        goal_to_return.target_pose.pose.position.y = y
        goal_to_return.target_pose.pose.position.z = 0.0

        (x_quat, y_quat, z_quat, w_quat) = quaternion_from_euler(0.0, 0.0, z)

        goal_to_return.target_pose.pose.orientation.x = x_quat
        goal_to_return.target_pose.pose.orientation.y = y_quat
        goal_to_return.target_pose.pose.orientation.z = z_quat
        goal_to_return.target_pose.pose.orientation.w = w_quat

        return goal_to_return
    
    def givePoseEstimate(self, initial_pose : PoseWithCovarianceStamped) -> None:

        self.initial_pose_estimate_pub.publish(initial_pose)
    
    def transitionCallback(self, goal_handle : actionlib.ClientGoalHandle) -> None:

        goal = goal_handle.get_goal_status()
        rospy.loginfo("Got transition state: %d", goal)
        
        if goal == GoalStatus.PENDING:
            rospy.loginfo("The goal has yet to be processed by the action server")

        if goal == GoalStatus.ACTIVE:
            rospy.loginfo("The goal is currently being processed by the action server")

        if goal == GoalStatus.PREEMPTED:
            rospy.logwarn("The goal received a cancel request after it started executing and has since completed its execution (Terminal State)")
            self.activeGoal = False

        if goal == GoalStatus.SUCCEEDED:
            rospy.loginfo("The goal was achieved successfully by the action server (Terminal State)")
            self.activeGoal = False

        if goal == GoalStatus.ABORTED:
            rospy.logerr("The goal was aborted during execution by the action server due to some failure (Terminal State)")
            self.activeGoal = False

        if goal == GoalStatus.REJECTED:
            rospy.logwarn("The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)")
            self.activeGoal = False

        if goal == GoalStatus.PREEMPTING:
            rospy.loginfo("The goal received a cancel request after it started executing and has not yet completed execution")

        if goal == GoalStatus.RECALLING:
            rospy.loginfo("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled")

        if goal == GoalStatus.RECALLED:
            rospy.logwarn("The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)")
            self.activeGoal = False

        if goal == GoalStatus.LOST:
            rospy.logwarn("Lost Goal")

    def feedbackCallback(self, goal_handle : actionlib.ClientGoalHandle, feedback : MoveBaseFeedback) -> None:

        (x, y ,z) = feedback.base_position.pose.position.x, feedback.base_position.pose.position.x, feedback.base_position.pose.position.x

        (roll, pitch, yaw) = euler_from_quaternion([feedback.base_position.pose.orientation.x, 
                                                   feedback.base_position.pose.orientation.y, 
                                                   feedback.base_position.pose.orientation.z, 
                                                   feedback.base_position.pose.orientation.w])
        
        current_time = rospy.Time.now()

        if current_time - self.latest_published_feedback > rospy.Duration(2):
            rospy.loginfo("Feedback received from the action server -> Current position is x: %f, y: %f, yaw: %f", x, y, yaw)
            self.latest_published_feedback = rospy.Time.now()

    def run(self):

        if not self.confirmServerUp:
            rospy.logerr("Action Server could not be reached..finishing execution")
            rospy.signal_shutdown("Action server not available. Please restart")
            return
        
        rospy.loginfo("Sending Initial Pose Estimate")
        start_pose = InitialPoseEstimate()
        self.givePoseEstimate(start_pose)

        time.sleep(5)
        
        while not rospy.is_shutdown():

            if not self.activeGoal:
                random_goal = self.generateRandomGoal()
                rospy.loginfo("Sending Goal to Action Server")
                goal_handle = self.goal_action_client.send_goal(goal=random_goal, feedback_cb=self.feedbackCallback, transition_cb=self.transitionCallback)
                self.activeGoal = True
            else:
                time.sleep(5)
            
if __name__ == '__main__':
    
    try:
        controller = MoveBaseController()
        controller.start()
        rospy.spin()
        controller.join()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller was finished due to Interruption")
    except Exception as e:
        rospy.loginfo("Unexpected Exception ocurred: %s", str(e))