#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import threading

class WanderController(threading.Thread):

    def __init__(self, minimum_acceptable_range: float) -> None:

        threading.Thread.__init__(self)
        rospy.init_node("python_wander_controller")
        self.sensor_scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.obstacle_ahead = False
        self.minimum_acceptable_range = minimum_acceptable_range
        self.minimum_range_ahead = None
        self.state_change_time = rospy.Time.now()
        self.rate = rospy.Rate(10)
        rospy.loginfo("Python Controller has been initialized")
        
    def scanCallback(self, msg : LaserScan):

        self.minimum_range_ahead = min(msg.ranges)

    def wanderFunction(self):

        while not rospy.is_shutdown():

            try:
            
                if not self.obstacle_ahead:
                    if (self.minimum_acceptable_range < self.minimum_acceptable_range or rospy.Time.now() > self.state_change_time):
                        self.obstacle_ahead = True
                        self.state_change_time = rospy.Time.now() + rospy.Duration(5)

                else:
                    if (rospy.Time.now() > self.state_change_time):
                        self.obstacle_ahead = False
                        self.state_change_time = rospy.Time.now() + rospy.Duration(30)

                twist_to_publish = Twist()

                if not self.obstacle_ahead:
                    twist_to_publish.linear.x = 1
                else:
                    twist_to_publish.angular.z = 1

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
    
    controller = WanderController(0.8)
    controller.start()
    rospy.spin()
    controller.join()

