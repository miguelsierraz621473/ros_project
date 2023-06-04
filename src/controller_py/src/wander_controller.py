#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import threading
import enum

class RotationDirection(enum.Enum):
    NOT_DEFINED = 0
    CLOCKWISE = 1
    COUNTERCLOCKWISE = 2

class WanderController(threading.Thread):

    def __init__(self, minimum_acceptable_range_ahead: float, minimum_acceptable_range_side: float) -> None:

        threading.Thread.__init__(self)
        rospy.init_node("python_wander_controller")

        self.sensor_scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        self.robot_odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.latest_odom_update_time = None
        self.latest_laser_scan_update_time = None
        
        self.minimum_acceptable_range = minimum_acceptable_range_ahead
        self.minimum_acceptable_space_sides = minimum_acceptable_range_side
        self.angle_delta = 30

        self.obstacle_ahead = False
        self.current_ranges = tuple()
        self.minimum_distance = None
        self.minimum_distance_angle = None
        self.min_distance_in_estimated_orientation = None
        self.rotation = RotationDirection.NOT_DEFINED

        self.estimated_velocity = None
        self.estimated_orientation = None

        self.rate = rospy.Rate(10)

        rospy.loginfo("Python Controller has been initialized")
        
    def scanCallback(self, msg : LaserScan) -> None:

        self.latest_laser_scan_update_time = rospy.Time.now()

        self.current_ranges = msg.ranges
        self.minimum_distance = min(self.current_ranges)
        self.minimum_distance_angle = self.current_ranges.index(self.minimum_distance)
        # rospy.loginfo("Values Minimum Range: %f Angle: %f Type: %s", self.minimum_distance, self.minimum_distance_angle, str(type(self.current_ranges)))

    def odomCallback(self, msg: Odometry) -> None:

        self.latest_odom_update_time = rospy.Time.now()

        self.estimated_velocity = msg.twist.twist.linear.x

        current_pose = msg.pose.pose
        (x, y, self.estimated_orientation) = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
        self.estimated_orientation = math.degrees(self.estimated_orientation)
        if self.estimated_orientation < 0:
            self.estimated_orientation = 360 - abs(self.estimated_orientation)

        self.estimated_orientation = int(self.estimated_orientation)

        if len(self.current_ranges) > 0:
            if self.estimated_orientation < 0 or self.estimated_orientation > 360:
                rospy.logwarn("Orientation out of range : %d", self.estimated_orientation)
            self.min_distance_in_estimated_orientation = self.current_ranges[self.estimated_orientation]

    def define_rotation_direction(self, min_distance_in_estimated_orientation: float, estimated_orientation: float, whole_ranges: tuple, angle_delta: int) -> RotationDirection:

        if min_distance_in_estimated_orientation is None or estimated_orientation is None or len(whole_ranges) == 0:
            return RotationDirection.NOT_DEFINED
        
        angle_right, angle_left = self.get_view_angles(estimated_orientation, angle_delta)

        distance_right = whole_ranges[angle_right]
        distance_left = whole_ranges[angle_left]

        if distance_left < min_distance_in_estimated_orientation < distance_right:
            return RotationDirection.CLOCKWISE
        elif distance_left > min_distance_in_estimated_orientation > distance_right:
            return RotationDirection.COUNTERCLOCKWISE
        else:
            return RotationDirection.NOT_DEFINED
        
    def check_safe_to_go_area(self, estimated_orientation: float, whole_ranges: tuple, angle_delta: int) -> bool:

        if estimated_orientation is None or len(whole_ranges) == 0:
            return False

        angle_right, angle_left = self.get_view_angles(estimated_orientation, angle_delta)

        distance_right = whole_ranges[angle_right]
        distance_left = whole_ranges[angle_left]

        if distance_left > self.minimum_acceptable_range and distance_right > self.minimum_acceptable_range:
            return True
        else:
            return False

    def get_view_angles(self, estimated_orientation: float, angle_delta: int) -> tuple:

        if estimated_orientation <= (360 - angle_delta):
            angle_right = estimated_orientation + angle_delta
        else:
            angle_right = 360 - estimated_orientation + angle_delta

        if estimated_orientation >= (angle_delta):
            angle_left = estimated_orientation - angle_delta
        else:
            angle_left = 360 - abs(estimated_orientation - angle_delta)

        return angle_right, angle_left

    def wanderFunction(self) -> None:

        while not rospy.is_shutdown():

            try:

                if self.min_distance_in_estimated_orientation is None or self.minimum_distance_angle is None or self.minimum_distance is None:
                    # print("Skipping")
                    continue

                print("CONDITION EVALUATION - Distance to obstacle ahead: ", self.min_distance_in_estimated_orientation, " Minimum Acceptable Range: " , self.minimum_acceptable_range, " Obstacle Ahead Judgement: ", self.obstacle_ahead)
            
                if not self.obstacle_ahead:
                    if (self.min_distance_in_estimated_orientation < self.minimum_acceptable_range):
                        self.obstacle_ahead = True

                else:
                    if (self.min_distance_in_estimated_orientation > self.minimum_acceptable_range and not self.check_safe_to_go_area(self.estimated_orientation, self.current_ranges, self.angle_delta)):
                        self.obstacle_ahead = False


                twist_to_publish = Twist()

                if not self.obstacle_ahead:
                    print("Advancing")
                    twist_to_publish.linear.x = 0.5
                    twist_to_publish.angular.z = 0.0
                else:
                    print("Rotating")
                    rotation_direction = self.define_rotation_direction(self.min_distance_in_estimated_orientation, self.estimated_orientation, self.current_ranges, self.angle_delta)
                    if rotation_direction == RotationDirection.CLOCKWISE or rotation_direction == RotationDirection.NOT_DEFINED:
                        sign = 1
                        # print("Clockwise")
                    elif rotation_direction == RotationDirection.COUNTERCLOCKWISE:
                        # print("Coutnerclockwise")
                        sign = -1

                    twist_to_publish.linear.x = 0.0
                    twist_to_publish.angular.z = sign*0.5

                self.twist_publisher.publish(twist_to_publish)

                self.rate.sleep()

            except Exception as e:
                rospy.logerr("Error has occured : %s", str(e))

            except KeyboardInterrupt:
                rospy.logwarn("Keyboard Interrup: Stop Publishing Commands")
                break

    def run(self):
        self.wanderFunction()
        
if __name__ == '__main__':
    
    controller = WanderController(0.8, 0.2)
    controller.start()
    rospy.spin()
    controller.join()

