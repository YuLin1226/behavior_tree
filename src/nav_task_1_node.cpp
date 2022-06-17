#include "../include/nav_bt_task/setup/nav_task_1.h"

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Action_NavForwardPath/>
            <Action_GoalReached/>
            <Action_NavBackwardPath/>
        </Sequence>
    </BehaviorTree>
</root>
 )";

using namespace BT;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"my_node_name");

    BT::BehaviorTreeFactory factory;

    NavTask::TaskOne task;

    // register all the actions into the factory
    NavTask::TaskOne::RegisterNodes(factory, &task);

    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    // This logger saves state changes on file
    FileLogger logger_file(tree, "nav_task_1_trace.fbl");

    // This logger stores the execution time of each node
    MinitraceLogger logger_minitrace(tree, "nav_task_1_trace.json");

#ifdef ZMQ_FOUND
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
#endif

    printTreeRecursively(tree.rootNode());

    const bool LOOP = ( argc == 2 && strcmp( argv[1], "loop") == 0);

    do
    {
        NodeStatus status = NodeStatus::RUNNING;
        // Keep on ticking until you get either a SUCCESS or FAILURE state
        while(status == NodeStatus::RUNNING)
        {
            // ros::spinOnce();
            status = tree.tickRoot();
            NavTask::SleepMS(1);   // optional sleep to avoid "busy loops"
        }
        NavTask::SleepMS(1000);
    }
    while(LOOP);

    // if(LOOP)
    // {
    //     for(auto i=0; i<5; i++)
    //     {
    //         std::cout << "\ntickRoot." << std::endl;
    //         NodeStatus status = NodeStatus::RUNNING;
    //         // Keep on ticking until you get either a SUCCESS or FAILURE state
    //         MyBT::SleepMS(1);   // optional sleep to avoid "busy loops"
    //         status = tree.tickRoot();
    //         MyBT::SleepMS(1000);
    //     }
    // }

    return 0;
}
