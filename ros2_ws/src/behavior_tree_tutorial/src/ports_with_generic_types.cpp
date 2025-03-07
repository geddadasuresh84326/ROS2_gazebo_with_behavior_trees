#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"

struct position2D
{
    double x;
    double y;
};

class CalculateGoal : public BT::SyncActionNode{
    public:
        CalculateGoal(const std::string& name,const BT::NodeConfig& config) : BT::SyncActionNode(name,config){}

        static BT::PortsList providedPorts(){
            return {BT::OutputPort<position2D>("goal")};
        }
        BT::NodeStatus tick() override{
            position2D my_goal = {1.1,2.3};
            setOutput<position2D>("goal",my_goal);
            return BT::NodeStatus::SUCCESS;
        }
};

class PrintTarget : public BT::SyncActionNode {
    public :
        PrintTarget(const std::string& name,const BT::NodeConfig& config): BT::SyncActionNode(name,config){}

        static BT::PortsList providedPorts(){
            const char* description = "Print the goal position";
            return {BT::InputPort<position2D>("target",description)};
        }
        BT::NodeStatus tick() override{
            auto res = getInput<position2D>("target");
            if (!res){
                throw BT::RuntimeError("Error reading port [target]: ",res.error());
            }
            position2D target = res.value();
            std::cout<<"Target position : ["<<target.x<<","<<target.y<<"]\n";
            return BT::NodeStatus::SUCCESS;
        }
};

int main(){
    BT::BehaviorTreeFactory factory;
    auto package_share_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");

    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    auto tree = factory.createTreeFromFile(package_share_path+"/bt_xml/ports_with_generic_types.xml");

    tree.tickWhileRunning();  // Execute the behavior tree

    return 0;
}