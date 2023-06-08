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
    """
    Enumeration class for the different zones of the map
    """

    NOT_DEFINED = -1
    AREA_RIGHT = 0
    AREA_UP = 1
    AREA_LEFT = 2
    AREA_DOWN = 3
    AREA_MIDDLE = 4

class AreaDelimitations():
    """
    Class that acts as a dictionary to save the regions for each area. Each vertex is defines as a shapely Point.
    """

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
        """
        """
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
        """
        A controller for moving a robot using the move_base action in ROS.

        This controller initializes the necessary ROS nodes, sets up the action client, subscribes to the odometry topic,
        and initializes other required attributes.

        Args:
            None

        Attributes:
            goal_action_client (ActionClient): The action client for the 'move_base' action.
            robot_odom_sub (Subscriber): The subscriber to the '/odom' topic for receiving odometry data.
            initial_pose_estimate_pub (Publisher): The publisher for the '/initialpose' topic to send initial pose estimates.
            current_estimated_pose (Odometry): The current estimated pose of the robot.
            current_area (GeneralArea): The current area of the robot's location.
            available_areas (dict): A dictionary of available areas and their associated polygons.
            latest_published_feedback (rospy.Time): The timestamp of the latest published feedback.
            activeGoal (bool): Flag indicating if there is an active goal being executed.

        Raises:
            None

        Example:
            controller = MoveBaseController()

        ROSのmove_baseアクションを使用してロボットを移動させるためのコントローラ。

        このコントローラは必要なROSノードを初期化し、アクションクライアントを設定し、オドメトリトピックにサブスクライブし、
        他の必要な属性を初期化します。

        引数:
            なし

        属性:
            goal_action_client (ActionClient): 'move_base'アクションのアクションクライアント。
            robot_odom_sub (Subscriber): オドメトリデータを受け取るための'/odom'トピックのサブスクライバ。
            initial_pose_estimate_pub (Publisher): '/initialpose'トピックに初期姿勢推定を送信するためのパブリッシャ。
            current_estimated_pose (Odometry): ロボットの現在の推定姿勢。
            current_area (GeneralArea): ロボットの位置の現在のエリア。
            available_areas (dict): 利用可能なエリアとそれに関連するポリゴンの辞書。
            latest_published_feedback (rospy.Time): 最新のフィードバックのタイムスタンプ。
            activeGoal (bool): 実行中のゴールがあるかどうかを示すフラグ。

        例:
            controller = MoveBaseController()

        """

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
        """
        Creates Shapely Polygons for each of the areas inside.

        This function iterates over the defined areas and their corresponding regions in the AreaDelimitations.AREAS dictionary.
        It creates a Shapely Polygon object for each region using the list of vertex points.

        Arguments:
            None

        Returns:
            None

        各エリア内のShapelyポリゴンを作成します。

        この関数は、AreaDelimitations.AREAS辞書内の定義されたエリアとそれに対応する領域を反復処理します。
        各領域に対して、頂点のリストを使用してShapelyのポリゴンオブジェクトを作成します。

        引数:
            なし

        戻り値:
            なし
        """     

        for area, regions in AreaDelimitations.AREAS.items():
            self.available_areas[area] = Polygon(list(regions.values()))
        
    def confirmServerUp(self) -> bool:
        """
        Confirms that the move_base action server is running.

        This function uses the `wait_for_server` method of the `goal_action_client` to check if the action server is up and running.
        It waits for a maximum of 5 seconds for the server to become available.

        Arguments:
            None

        Returns:
            bool: True if the action server is running, False otherwise.


        move_baseアクションサーバーが稼働していることを確認します。

        この関数は、`goal_action_client`の`wait_for_server`メソッドを使用して、アクションサーバーが起動しているかどうかを確認します。
        最大5秒間、サーバーが利用可能になるまで待機します。

        引数:
            なし

        戻り値:
            bool: アクションサーバーが稼働している場合はTrue、それ以外の場合はFalse。
        """

        return self.goal_action_client.wait_for_server(rospy.Duration(5))

    def isInsideArea(self, point : Point, polygon: Polygon) -> bool:
        """
        Checks if a point is inside a polygon.

        Arguments:
            point (Point): The 2D point representing a coordinate on the map. This data is typically obtained from the odometry and represents the actual position of the robot.
            polygon (Polygon): The Shapely polygon object defined by its vertices.

        Returns:
            bool: True if the point is inside the polygon, False otherwise.

        ポリゴンの内部に指定の座標が含まれているかどうかをチェックします。

        引数:
            point (Point): マップ上の座標を表す2Dポイント。このデータは通常、オドメトリから取得され、ロボットの実際の位置を表します。
            polygon (Polygon): 頂点によって定義されたShapelyのポリゴンオブジェクト。

        戻り値:
            bool: 指定の座標がポリゴンの内部にある場合はTrue、それ以外の場合はFalse。
        """

        if polygon.contains(point):
            return True
        else:
            return False

    def determineCurrentArea(self, x: float, y: float) -> GeneralArea:
        """
        Determines the current area based on the given coordinates.

        Arguments:
            x (float): The x coordinate for the robot's position.
            y (float): The y coordinate for the robot's position.

        Returns:
            GeneralArea: The current area determined based on the coordinates.

        Raises:
            None

        座標に基づいて現在のエリアを判定します。

        引数:
            x (float): ロボットの位置のx座標。
            y (float): ロボットの位置のy座標。

        戻り値:
            GeneralArea: 座標に基づいて判定された現在のエリア。

        例外:
            なし
        """

        for area, polygon in self.available_areas.items():
            if self.isInsideArea(Point(x, y), polygon):
                return GeneralArea[area]
            
        return GeneralArea.NOT_DEFINED

    def odomCallback(self, msg: Odometry) -> None:
        """
        Callback function for the '/odom' topic that updates the current estimated pose and area.

        Arguments:
            msg (Odometry): The Odometry message containing the robot's pose and position information.

        Returns:
            None

        Raises:
            None

        '/odom' トピックのコールバック関数で、現在の推定姿勢とエリアを更新します。

        引数:
            msg (Odometry): ロボットの姿勢と位置情報を含む Odometry メッセージ。

        戻り値:
            None

        例外:
            なし
        """

        self.current_estimated_pose = msg
        self.current_area = self.determineCurrentArea(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def selectRandomArea(self) -> GeneralArea:

        """
        Selects a random area from the available areas.

        This function randomly selects an area from the list of available areas while ensuring that the selected area is not the same as the current area.
        It uses a time-based loop to limit the selection process to 10 seconds.

        Returns:
            GeneralArea: The randomly selected area.

        Raises:
            None

        利用可能なエリアからランダムにエリアを選択します。

        この関数は、利用可能なエリアのリストからランダムにエリアを選択しますが、選択されたエリアが現在のエリアと異なることを確認します。
        選択プロセスを10秒に制限するために、時間ベースのループを使用します。

        戻り値:
            GeneralArea: ランダムに選択されたエリア。

        例外:
            なし
        """


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
        """
        Generates a random point within a specified polygon.

        Arguments:
            desired_polygon (Polygon): The polygon object representing the desired shape.

        Returns:
            tuple: A tuple containing the randomly generated (x, y) coordinates within the polygon.

        Example:
            >>> polygon = Polygon([(0, 0), (0, 5), (5, 5), (5, 0)])
            >>> generate_random_point_within_polygon(polygon)
            (2.3, 3.8)

        指定されたポリゴン内でランダムな点を生成します。

        引数:
            desired_polygon (Polygon): ポリゴン形状を表すポリゴンオブジェクト。

        戻り値:
            tuple: ポリゴン内でランダムに生成された (x, y) 座標を含むタプル。

        例:
            >>> polygon = Polygon([(0, 0), (0, 5), (5, 5), (5, 0)])
            >>> generate_random_point_within_polygon(polygon)
            (2.3, 3.8)
        """

        xmin, ymin, xmax, ymax = desired_polygon.bounds

        while True:

            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            if Point(x, y).within(desired_polygon):
                break

        return (x, y)
    
    def selectRandomOrientation(self) -> float:
        """
        Generates a random orientation angle within the range of 0 to 360 degrees.

        Returns:
            float: A random orientation angle in degrees.

        Example:
            >>> selectRandomOrientation()
            245.8

        0度から360度の範囲内でランダムな方向角を生成します。

        戻り値:
            float: ランダムな方向角（度数法）。

        例:
            >>> selectRandomOrientation()
            245.8

        """

        return float(random.randint(0, 360))

    def generateRandomGoal(self) -> MoveBaseGoal:

        """
        Generates a random goal for the move_base action.

        Returns:
            MoveBaseGoal: A MoveBaseGoal object representing the randomly generated goal.

        Example:
            >>> generateRandomGoal()
            MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id='map', stamp=rospy.Time(0)), pose=Pose(position=Point(x=2.3, y=3.8, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.987, w=0.161))))
        ランダムなゴールをmove_baseアクションのために生成します。

        戻り値:
            MoveBaseGoal: ランダムに生成されたゴールを表すMoveBaseGoalオブジェクト。

        例:
            >>> generateRandomGoal()
            MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id='map', stamp=rospy.Time(0)), pose=Pose(position=Point(x=2.3, y=3.8, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.987, w=0.161))))
        
        """

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
        """
        Publishes the initial pose estimate.

        Args:
            initial_pose (PoseWithCovarianceStamped): The initial pose estimate to be published.

        Returns:
            None

        初期姿勢の推定値をパブリッシュします。

        引数:
            initial_pose (PoseWithCovarianceStamped): パブリッシュする初期姿勢の推定値。

        戻り値:
            None
        """

        self.initial_pose_estimate_pub.publish(initial_pose)
    
    def transitionCallback(self, goal_handle : actionlib.ClientGoalHandle) -> None:
        """
        Callback function to handle goal transition states.

        Args:
            goal_handle (actionlib.ClientGoalHandle): The goal handle containing the transition state.

        Returns:
            None

        ゴールの遷移状態を処理するためのコールバック関数です。

        引数:
            goal_handle (actionlib.ClientGoalHandle): 遷移状態を含むゴールハンドル。

        戻り値:
            None
        """

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
        """
        Callback function to handle feedback received from the action server.

        Args:
            goal_handle (actionlib.ClientGoalHandle): The goal handle associated with the feedback.
            feedback (MoveBaseFeedback): The feedback received from the action server.

        Returns:
            None

        アクションサーバから受け取ったフィードバックを処理するためのコールバック関数です。

        引数:
            goal_handle (actionlib.ClientGoalHandle): フィードバックに関連するゴールハンドル。
            feedback (MoveBaseFeedback): アクションサーバから受け取ったフィードバック。

        戻り値:
            None
        """

        (x, y ,z) = feedback.base_position.pose.position.x, feedback.base_position.pose.position.x, feedback.base_position.pose.position.x

        (roll, pitch, yaw) = euler_from_quaternion([feedback.base_position.pose.orientation.x, 
                                                   feedback.base_position.pose.orientation.y, 
                                                   feedback.base_position.pose.orientation.z, 
                                                   feedback.base_position.pose.orientation.w])
        
        current_time = rospy.Time.now()

        if current_time - self.latest_published_feedback > rospy.Duration(2):
            rospy.loginfo("Feedback received from the action server -> Current position is x: %f, y: %f, yaw: %f", x, y, yaw)
            self.latest_published_feedback = rospy.Time.now()

    def run(self) -> None:
        """
        Executes the main functionality of the controller.

        This function confirms the availability of the action server, sends the initial pose estimate,
        and continuously generates and sends random goals to the action server while the program is running.

        Args:
            None

        Returns:
            None

        コントローラのメイン機能を実行します。

        この関数はアクションサーバの利用可能性を確認し、初期姿勢の推定を送信し、
        プログラムが実行中の間、連続的にランダムなゴールを生成してアクションサーバに送信します。

        引数:
            None

        戻り値:
            None
        """

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
    """
    The main entry point of the program.

    This function initializes the MoveBaseController, starts its execution, and enters the ROS event loop.
    It handles any ROS interruption exceptions and logs any unexpected exceptions that may occur.

    Args:
        None

    Returns:
        None

    プログラムのメインエントリーポイントです。

    この関数はMoveBaseControllerを初期化し、実行を開始し、ROSのイベントループに入ります。
    ROSの割り込み例外を処理し、予期しない例外が発生した場合はログに記録します。

    引数:
        None

    戻り値:
        None
    """
    
    try:
        controller = MoveBaseController()
        controller.start()
        rospy.spin()
        controller.join()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller was finished due to Interruption")
    except Exception as e:
        rospy.loginfo("Unexpected Exception ocurred: %s", str(e))