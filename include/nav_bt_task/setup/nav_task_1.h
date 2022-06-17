#pragma once
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <ros/callback_queue.h>


using namespace BT;
namespace NavTask
{
    inline void SleepMS(int ms)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    // BT::NodeStatus actionNavForwardPath();
    // BT::NodeStatus actionNavBackwardPath();
    // BT::NodeStatus actionPrintReachGoal();

    // void RegisterNodes(BT::BehaviorTreeFactory& factory);

    class TaskOne
    {
    private:
        static const int NUM_THREADS = 2;
        ros::NodeHandle private_nh_;
        // ros::AsyncSpinner asyncSpinner_;
        // ros::CallbackQueue callbackQueue_;
        ros::Publisher waypoints_pub_;
        ros::Subscriber nav_goal_result_sub_;
        bool initialized_;
        nav_msgs::Path waypoints_;
        void subCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result);
        bool navSucceed_;
    public:
        TaskOne();
        ~TaskOne();
        void initialize();
        void pubForwardPath();
        void pubBackwardPath();
        void addWaypointToVector(double pointX, double pointY, double pointYAW);

        NodeStatus actionNavForwardPath();
        NodeStatus actionNavBackwardPath();
        NodeStatus actionPrintReachGoal();
        static void RegisterNodes(BehaviorTreeFactory& factory, TaskOne* TaskOne);
    };
    
}
