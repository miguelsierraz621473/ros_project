#ifndef WANDERER_CONTROLLER_H
#define WANDERER_CONTROLLER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


class WanderController
{
    public:
        WanderController(ros::NodeHandle* nodehandle, float minimum_acceptable_range);
        ~WanderController();

        void run();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sensor_scan_sub_;
        ros::Publisher twist_publisher_;

        bool obstacle_ahead_;
        float minimum_acceptable_range_;
        float minimum_range_ahead_;
        ros::Time state_change_time_;

        void scanCallback(const sensor_msgs::LaserScan msg_holder);
        void wanderFunction();
};

#endif // WANDERER_CONTROLLER_H