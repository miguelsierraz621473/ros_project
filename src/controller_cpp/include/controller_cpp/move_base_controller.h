#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <thread>
#include <map>
#include <tuple>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <geos/geom/Point.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/geom/CoordinateSequence.h>

enum GeneralArea
{
    NOT_DEFINED=0,
    AREA_RIGHT=1,
    AREA_UP=2,
    AREA_LEFT=3,
    AREA_DOWN=4,
    AREA_MIDDLE=5
};

using namespace move_base_msgs;
using namespace geos::geom;
typedef std::map<std::string, std::unique_ptr<Point>> PointDefinition;
typedef std::map<GeneralArea, PointDefinition> AreaDefinition;
typedef std::map<GeneralArea, std::unique_ptr<Polygon>> PolygonDefinition;

class MoveBaseController
{
    public:
        MoveBaseController(ros::NodeHandle* nodehandle);
        ~MoveBaseController();

        std::unique_ptr<PrecisionModel> pm;
        GeometryFactory::Ptr factory;

        void run();

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionClient<MoveBaseAction> goal_action_client;
        ros::Subscriber robot_odom_sub;
        ros::Publisher initial_pose_estimate_pub;

        geometry_msgs::PoseWithCovarianceStamped initial_pose_estimate;

        nav_msgs::Odometry current_estimated_pose;

        GeneralArea current_area;
        AreaDefinition defined_areas;
        PolygonDefinition available_areas;

        ros::Time latest_published_feedback;
        
        bool activeGoal;

        void createPolygons();
        void defineInitialPoseEstimate();
        bool confirmServerUp();
        bool isInsideArea(geos::geom::Point* point, geos::geom::Polygon* polygon);
        GeneralArea determineCurrentArea(float x, float y);
        void odomCallback(const nav_msgs::Odometry msg_holder);
        GeneralArea selectRandomArea();
        std::tuple<float, float> selectRandomCoordinatesFromArea(GeneralArea desired_area);
        float selectRandomOrientation();
        MoveBaseGoal generateRandomGoal();
        void givePoseEstimate(geometry_msgs::PoseWithCovarianceStamped initial_pose);

        void activeCallback();
        void feedbackCallback(const MoveBaseActionFeedbackConstPtr& feedback);
        void doneCallback(const actionlib::SimpleClientGoalState& state, 
                          const MoveBaseActionResultConstPtr& result);
};