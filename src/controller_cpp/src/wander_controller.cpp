#include "controller_cpp/wander_controller.h"

WanderController::WanderController(ros::NodeHandle* nodehandle, float minimum_acceptable_range) : nh_(*nodehandle)
{
    this->sensor_scan_sub_ = nh_.subscribe("/scan", 10, &WanderController::scanCallback, this);
    this->twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    this->obstacle_ahead_ = false;
    this->minimum_acceptable_range_ = minimum_acceptable_range;
    this->state_change_time_ = ros::Time::now();

    ROS_INFO("C++ Controller has been initialized");

};

WanderController::~WanderController()
{
    ROS_WARN("C++ Controller has been stopped");
};

void WanderController::scanCallback(const sensor_msgs::LaserScan msg_holder)
{
    
    this->minimum_range_ahead_ = msg_holder.range_min;
};

void WanderController::wanderFunction()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!this->obstacle_ahead_)
        {
            auto current_time = ros::Time::now();
            if (this->minimum_range_ahead_ < this->minimum_acceptable_range_ || current_time > this->state_change_time_)
            {
                this->obstacle_ahead_ = true;
                this->state_change_time_ = ros::Time::now() + ros::Duration(5);
            }
        }
        else
        {
            auto current_time = ros::Time::now();
            if (current_time > this->state_change_time_)
            {
                this->obstacle_ahead_ = true;
                this->state_change_time_ = ros::Time::now() + ros::Duration(30);
            }
        }

        geometry_msgs::Twist twist_to_publish = geometry_msgs::Twist();
        
        if (!this->obstacle_ahead_) {twist_to_publish.linear.x = 1;}
        else {twist_to_publish.angular.z = 1;}

        this->twist_publisher_.publish(twist_to_publish);

        loop_rate.sleep();

    }
};

void WanderController::run()
{
    this->wanderFunction();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_wander_controller");
    ros::NodeHandle nh;
    WanderController wander_controller(&nh, float(0.8));
    wander_controller.run();
    return 0;
}