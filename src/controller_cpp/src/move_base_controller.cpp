#include "controller_cpp/move_base_controller.h"

MoveBaseController::MoveBaseController(ros::NodeHandle* nodehandle) : nh_(*nodehandle), goal_action_client("move_base", true), pm(new PrecisionModel())
{

    this->robot_odom_sub = nh_.subscribe("/odom", 1, &MoveBaseController::odomCallback, this);
    this->initial_pose_estimate_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    this->current_area = NOT_DEFINED;
    this->createPolygons();
    this->defineInitialPoseEstimate();

    this->latest_published_feedback = ros::Time::now();

    this->activeGoal = false;

    this->factory = GeometryFactory::create(this->pm.get(), -1);

    while (!this->goal_action_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Successfully initialized controller");
}

MoveBaseController::~MoveBaseController()
{
    ROS_WARN("Controller was destroyed");
}

void MoveBaseController::defineInitialPoseEstimate()
{
    this->initial_pose_estimate.header.frame_id = "map";
    this->initial_pose_estimate.header.stamp = ros::Time::now();

    this->initial_pose_estimate.pose.pose.position.x = -1.9900001287460327;
    this->initial_pose_estimate.pose.pose.position.y = -0.45999985933303833;
    this->initial_pose_estimate.pose.pose.position.z = 0.0;

    this->initial_pose_estimate.pose.pose.orientation.x = 0.0;
    this->initial_pose_estimate.pose.pose.orientation.y = 0.0;
    this->initial_pose_estimate.pose.pose.orientation.z = -4.139211284837073e-08;
    this->initial_pose_estimate.pose.pose.orientation.w = 0.9999999999999991;
}

void MoveBaseController::createPolygons()
{
    PointDefinition a1;
 
    Coordinate v_p1_a1(1.1313250064849854, 2.4731078147888184);
    Coordinate v_p2_a1(1.4579946994781494, 1.964996576309204);
    Coordinate v_p3_a1(1.4172468185424805, 1.2950527667999268);
    Coordinate v_p4_a1(-1.3090224266052246, 1.3626840114593506);
    Coordinate v_p5_a1(-1.305069088935852, 2.0217161178588867);
    Coordinate v_p6_a1(-1.0252100229263306, 2.5042612552642822);
    a1["P1"] = this->factory->createPoint(v_p1_a1);
    a1["P2"] = this->factory->createPoint(v_p2_a1);
    a1["P3"] = this->factory->createPoint(v_p3_a1);
    a1["P4"] = this->factory->createPoint(v_p4_a1);
    a1["P5"] = this->factory->createPoint(v_p5_a1);
    a1["P6"] = this->factory->createPoint(v_p6_a1);

    std::initializer_list<Coordinate> list1 = {v_p1_a1, v_p2_a1, v_p3_a1, v_p4_a1, v_p5_a1, v_p6_a1};
    
    PointDefinition a2;
    Coordinate v_p1_a2(1.4591257572174072, 1.908069133758545);
    Coordinate v_p2_a2(1.84257173538208, 1.9523131847381592);
    Coordinate v_p3_a2(2.611079692840576, 0.5322036147117615);
    Coordinate v_p4_a2(2.347233772277832, -0.022695258259773254);
    Coordinate v_p5_a2(2.567981243133545, -0.5458083152770996);
    Coordinate v_p6_a2(1.7939517498016357, -1.9409345388412476);
    Coordinate v_p7_a2(1.3767004013061523, -1.9720466136932373);
    a2["P1"] = this->factory->createPoint(v_p1_a2);
    a2["P2"] = this->factory->createPoint(v_p2_a2);
    a2["P3"] = this->factory->createPoint(v_p3_a2);
    a2["P4"] = this->factory->createPoint(v_p4_a2);
    a2["P5"] = this->factory->createPoint(v_p5_a2);
    a2["P6"] = this->factory->createPoint(v_p6_a2);
    a2["P7"] = this->factory->createPoint(v_p7_a2);

    std::initializer_list<Coordinate> list2 = {v_p1_a2, v_p2_a2, v_p3_a2, v_p4_a2, v_p5_a2, v_p6_a2, v_p7_a2};

    PointDefinition a3;
    Coordinate v_p1_a3(1.3434334993362427, -2.012788772583008);
    Coordinate v_p2_a3(1.3762962818145752, -1.2581532001495361);
    Coordinate v_p3_a3(-1.3986012935638428, -1.2454938888549805);
    Coordinate v_p4_a3(-1.398937702178955, -1.8853110074996948);
    Coordinate v_p5_a3(-1.0495669841766357, -2.4744865894317627);
    Coordinate v_p6_a3(1.098577857017517, -2.4462695121765137);
    a3["P1"] = this->factory->createPoint(v_p1_a3);
    a3["P2"] = this->factory->createPoint(v_p2_a3);
    a3["P3"] = this->factory->createPoint(v_p3_a3);
    a3["P4"] = this->factory->createPoint(v_p4_a3);
    a3["P5"] = this->factory->createPoint(v_p5_a3);
    a3["P6"] = this->factory->createPoint(v_p6_a3);

    std::initializer_list<Coordinate> list3 = {v_p1_a3, v_p2_a3, v_p3_a3, v_p4_a3, v_p5_a3, v_p6_a3};

    PointDefinition a4;
    Coordinate v_p1_a4(-1.3720011711120605, -1.88504159450531);
    Coordinate v_p2_a4(-1.7487800121307373, -1.9224839210510254);
    Coordinate v_p3_a4(-2.832045555114746, -0.007140785455703735);
    Coordinate v_p4_a4(-1.6800928115844727, 1.9844343662261963);
    Coordinate v_p5_a4(-1.3233144283294678, 2.001471996307373);
    a4["P1"] = this->factory->createPoint(v_p1_a4);
    a4["P2"] = this->factory->createPoint(v_p2_a4);
    a4["P3"] = this->factory->createPoint(v_p3_a4);
    a4["P4"] = this->factory->createPoint(v_p4_a4);
    a4["P5"] = this->factory->createPoint(v_p5_a4);

    std::initializer_list<Coordinate> list4 = {v_p1_a4, v_p2_a4, v_p3_a4, v_p4_a4, v_p5_a4};

    PointDefinition a5;
    Coordinate v_p1_a5(-1.2015597820281982, 1.2753223180770874);
    Coordinate v_p2_a5(-1.2302465438842773, -1.2438101768493652);
    Coordinate v_p3_a5(1.2480765581130981, -1.2324957847595215);
    Coordinate v_p4_a5(1.2504326105117798, 1.2257593870162964);
    a5["P1"] = this->factory->createPoint(v_p1_a5);
    a5["P2"] = this->factory->createPoint(v_p2_a5);
    a5["P3"] = this->factory->createPoint(v_p3_a5);
    a5["P4"] = this->factory->createPoint(v_p4_a5);

    std::initializer_list<Coordinate> list5 = {v_p1_a5, v_p2_a5, v_p3_a5, v_p4_a5};

    this->defined_areas[AREA_RIGHT] = a1;
    this->defined_areas[AREA_UP] = a2;
    this->defined_areas[AREA_LEFT] = a3;
    this->defined_areas[AREA_RIGHT] = a4;
    this->defined_areas[AREA_MIDDLE] = a5;

    this->available_areas[AREA_RIGHT] = this->factory->createPolygon(list1);
    this->available_areas[AREA_UP] = this->factory->createPolygon(list2);
    this->available_areas[AREA_LEFT] = this->factory->createPolygon(list3);
    this->available_areas[AREA_RIGHT] = this->factory->createPolygon(list4);
    this->available_areas[AREA_MIDDLE] = this->factory->createPolygon(list5);

}

bool MoveBaseController::confirmServerUp()
{
    return this->goal_action_client.waitForResult(ros::Duration(5.0));
}

bool MoveBaseController::isInsideArea(geos::geom::Point* point, geos::geom::Polygon* polygon)
{
    if (polygon->contains(point)) {return true;}
    else {return false;}
}

GeneralArea MoveBaseController::determineCurrentArea(float x, float y)
{
    Coordinate coord(x, y);
    std::unique_ptr<Point> p = this->factory->createPoint(coord);

    for (auto const &key_pair : this->available_areas)
    {
        if (this->isInsideArea(p.get(), key_pair.second.get())) 
        {
            return key_pair.first;
        }
    }

    return NOT_DEFINED;

}

void MoveBaseController::odomCallback(const nav_msgs::Odometry msg_holder)
{
    this->current_estimated_pose;
    this->current_area = this->determineCurrentArea(msg_holder.pose.pose.position.x, msg_holder.pose.pose.position.y);
}

GeneralArea MoveBaseController::selectRandomArea()
{
    GeneralArea random_area = NOT_DEFINED;
    ros::Time start_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    while ((random_area == NOT_DEFINED) &&(current_time-start_time < ros::Duration(10)))
    {
        int random_integer = rand() % 5 + 0;
        GeneralArea random_area = static_cast<GeneralArea>(random_integer);
        if (random_area != NOT_DEFINED && random_area != this->current_area)
        {
            break;
        }
    }

    if (random_area == NOT_DEFINED)
    {
        random_area = AREA_MIDDLE;
    }

    return random_area;
}

std::tuple<float, float> MoveBaseController::selectRandomCoordinatesFromArea(GeneralArea desired_area)
{
    float min_radius = 0.0;
    float max_radius = 2.3;
    int min_angle = 0;
    int max_angle = 360;

    if (desired_area == AREA_MIDDLE)
    {
        float min_radius = 0.0;
        float max_radius = 1.0;
    }
    else
    {
        float min_radius = 1.0;
        float max_radius = 2.3;
    }

    switch (desired_area)
    {
        case AREA_RIGHT:
            int min_angle = -45;
            int max_angle = 45;
            break;
        case AREA_UP:
            int min_angle = 45;
            int max_angle = 135;
            break;
        case AREA_LEFT:
            int min_angle = 135;
            int max_angle = 225;
            break;
        case AREA_DOWN:
            int min_angle = 225;
            int max_angle = 315;
            break;
        default:
            int min_angle = 0;
            int max_angle = 360;
            break;
    }

    float r =  min_radius + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_radius-min_radius)));
    float theta = (rand() % max_angle + min_angle) * (M_PI / 180.0);

    float x = r * cos(theta);
    float y = r * sin(theta);

    std::tuple<float, float> to_return;

    to_return = std::make_tuple(x, y);

    return to_return;
}

float MoveBaseController::selectRandomOrientation()
{
    return static_cast <float> (rand() % 360 + 0);
}

MoveBaseGoal MoveBaseController::generateRandomGoal()
{
    GeneralArea area_to_go = this->selectRandomArea();
    std::tuple<float, float> coords = this->selectRandomCoordinatesFromArea(area_to_go);
    float z = this->selectRandomOrientation();

    // ROS_INFO("Goal pose -> Selected area: %s -> x: %f y: %f z: %f", area_to_go.name, x, y, z);

    MoveBaseGoal goal_to_return;

    goal_to_return.target_pose.header.frame_id = "map";
    goal_to_return.target_pose.header.stamp = ros::Time::now();

    goal_to_return.target_pose.pose.position.x = std::get<0>(coords);
    goal_to_return.target_pose.pose.position.y = std::get<1>(coords);
    goal_to_return.target_pose.pose.position.z = 0.0;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, z);

    goal_to_return.target_pose.pose.orientation.x = quat.getX();
    goal_to_return.target_pose.pose.orientation.y = quat.getY();
    goal_to_return.target_pose.pose.orientation.z = quat.getZ();
    goal_to_return.target_pose.pose.orientation.w = quat.getW();

    return goal_to_return;

}

void MoveBaseController::givePoseEstimate(geometry_msgs::PoseWithCovarianceStamped initial_pose)
{
    this->initial_pose_estimate_pub.publish(initial_pose);
}

void MoveBaseController::activeCallback()
{
    ROS_INFO("The goal is currently being processed by the action server");
    this->activeGoal = true;
}

void MoveBaseController::feedbackCallback(const MoveBaseActionFeedbackConstPtr& feedback)
{
    float x = feedback->feedback.base_position.pose.position.x;
    float y = feedback->feedback.base_position.pose.position.y;

    float quat_x = feedback->feedback.base_position.pose.orientation.x;
    float quat_y = feedback->feedback.base_position.pose.orientation.y;
    float quat_z = feedback->feedback.base_position.pose.orientation.z;
    float quat_w = feedback->feedback.base_position.pose.orientation.x;

    tf::Quaternion full_quat(quat_x, quat_y, quat_z, quat_w);

    tf::Matrix3x3 rpy_vector(full_quat);
    double roll, pitch, yaw;
    rpy_vector.getRPY(roll, pitch, yaw);

    ros::Time current_time = ros::Time::now();

    if (current_time - this->latest_published_feedback > ros::Duration(2))
    {
        // ROS_INFO(("Feedback received from the action server -> Current position is x: %f, y: %f, yaw: %f", x, y, yaw);
        this->latest_published_feedback = ros::Time::now();
    }
}

void MoveBaseController::doneCallback(const actionlib::SimpleClientGoalState& state, const MoveBaseActionResultConstPtr& result)
{
    switch (state.state_)
    {
    case state.PENDING:
        ROS_INFO("The goal has yet to be processed by the action server");
        break;
    case state.ACTIVE:
        ROS_INFO("The goal is currently being processed by the action server");
        this->activeGoal = true;
        break;
    case state.RECALLED:
        ROS_INFO("The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)");
        this->activeGoal = false;
        break;
    case state.REJECTED:
        ROS_WARN("The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)");
        this->activeGoal = false;
        break;
    case state.PREEMPTED:
        ROS_WARN("The goal received a cancel request after it started executing and has since completed its execution (Terminal State)");
        this->activeGoal = false;
        break;
    case state.ABORTED:
        ROS_ERROR("The goal was aborted during execution by the action server due to some failure (Terminal State)");
        this->activeGoal = false;
        break;
    case state.SUCCEEDED:
        ROS_INFO("The goal was achieved successfully by the action server (Terminal State)");
        this->activeGoal = false;
        break;
    case state.LOST:
        ROS_WARN("Lost Goal");
        break;
    }
}

void MoveBaseController::run()
{
    if (!this->confirmServerUp())
    {
        ROS_INFO("Action Server could not be reached..finishing execution");
    }

    ROS_INFO("Sending Initial Pose Estimate");

    this->givePoseEstimate(this->initial_pose_estimate);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (ros::ok())     
    {
        if (!this->activeGoal)
        {
            MoveBaseGoal random_goal = this->generateRandomGoal();
            this->goal_action_client.sendGoal(random_goal, &MoveBaseController::doneCallback, &MoveBaseController::activeCallback, &MoveBaseController::feedbackCallback);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        ros::spinOnce();

    }   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_move_base_controller");
    ros::NodeHandle nh;
    MoveBaseController move_base_controller(&nh);
    move_base_controller.run();
    return 0;
}