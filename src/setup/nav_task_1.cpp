#include "../../include/nav_bt_task/setup/nav_task_1.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Quaternion.h"


// BT_REGISTER_NODES(factory)
// {
//     NavTask::TaskOne::RegisterNodes(factory);
// }

namespace NavTask
{
    // Action Declaration.
    NodeStatus TaskOne::actionNavForwardPath()
    {
        ROS_INFO("Forward path navigation.");
        pubForwardPath();
        return NodeStatus::SUCCESS;
    }

    NodeStatus TaskOne::actionNavBackwardPath()
    {
        ROS_INFO("Backward path navigation.");
        pubBackwardPath();
        return NodeStatus::SUCCESS;
    }

    NodeStatus TaskOne::actionPrintReachGoal()
    {
        std::cout << navSucceed_ << std::endl;
        ros::spin();
        while(!navSucceed_)
        {
            // if(!navSucceed_)
            // {
            //     ROS_INFO("Goal reached.");
            //     return NodeStatus::SUCCESS;   
            // }
            // ros::spin();
        }
        ROS_INFO("Goal reached.");
        return NodeStatus::SUCCESS;
    }

    void TaskOne::RegisterNodes(BehaviorTreeFactory& factory, NavTask::TaskOne* TaskOne)
    {
        factory.registerSimpleAction("Action_NavForwardPath", std::bind(&TaskOne::actionNavForwardPath, *TaskOne));
        factory.registerSimpleAction("Action_NavBackwardPath", std::bind(&TaskOne::actionNavBackwardPath, *TaskOne));
        factory.registerSimpleAction("Action_GoalReached", std::bind(&TaskOne::actionPrintReachGoal, *TaskOne));
    }
    // 
    TaskOne::TaskOne():
    initialized_(false),
    private_nh_("~"),
    // callbackQueue_(true),
    // asyncSpinner_(NUM_THREADS, &callbackQueue_),
    navSucceed_(false)
    {
        initialize();
    }

    TaskOne::~TaskOne()
    {
    }

    void TaskOne::initialize()
    {
        if(!initialized_)
        {
            // do initialization
            // private_nh_.setCallbackQueue(&callbackQueue_);
            // asyncSpinner_.start();
            waypoints_pub_ = private_nh_.advertise<nav_msgs::Path>("/global_way_points_array", 1);
            nav_goal_result_sub_ = private_nh_.subscribe("/waypoints_received", 10, &TaskOne::subCallback, this);
            ROS_INFO("Initialize Nav Task 1 successfully.");
            initialized_ = true;
        }
        else
        {
            ROS_WARN("Nav Task 1 has been initialized.");
        }
    }

    void TaskOne::pubForwardPath()
    {
        navSucceed_ = false;
        waypoints_.poses.clear();
        addWaypointToVector(1.0, 1.0, 0.0);
        addWaypointToVector(2.0, 2.0, 0.0);
        // Refresh vector header.
        waypoints_.header = waypoints_.poses.back().header;
        // pub
        waypoints_pub_.publish(waypoints_);
    }

    void TaskOne::pubBackwardPath()
    {
        navSucceed_ = false;
        waypoints_.poses.clear();
        addWaypointToVector(1.0, 1.0, 0.0);
        addWaypointToVector(0.0, 0.0, 0.0);
        // Refresh vector header.
        waypoints_.header = waypoints_.poses.back().header;
        // pub
        waypoints_pub_.publish(waypoints_);
    }

    void TaskOne::addWaypointToVector(double pointX, double pointY, double pointYAW)
    {
        ROS_INFO("Add waypoint to vector (x: %f | y: %f | yaw: %f).", pointX, pointY, pointYAW);
        tf2::Quaternion q;
        // Create this quaternion from roll/pitch/yaw (in radians)
        q.setRPY(0, 0, pointYAW);  
        q.normalize();
        
        geometry_msgs::PoseStamped waypoint;
        waypoints_.poses.push_back(waypoint);
        if(waypoints_.poses.size() > 1)
            waypoints_.poses.back().header.seq = waypoints_.poses.end()[-2].header.seq + 1;
        else
            waypoints_.poses.back().header.seq = 0;
        waypoints_.poses.back().header.stamp = ros::Time::now();
        waypoints_.poses.back().header.frame_id = "map";

        waypoints_.poses.back().pose.position.x = pointX;
        waypoints_.poses.back().pose.position.y = pointY;
        waypoints_.poses.back().pose.position.z = 0.0;
        waypoints_.poses.back().pose.orientation.x = q.getX();
        waypoints_.poses.back().pose.orientation.y = q.getY();
        waypoints_.poses.back().pose.orientation.z = q.getZ();
        waypoints_.poses.back().pose.orientation.w = q.getW();
    }

    void TaskOne::subCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
    {
        ROS_INFO("Get result: %i", result->status.status);
        if(result->status.status == 3)
            navSucceed_ = true;
        else
            navSucceed_ = false;
    }

}// namespace