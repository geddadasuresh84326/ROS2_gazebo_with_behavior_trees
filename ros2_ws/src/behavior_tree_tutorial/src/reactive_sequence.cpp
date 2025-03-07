#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace chr = std::chrono;

struct Pose2D
{
    double x,y,theta;
};


class MoveBaseAction : public BT::StatefulActionNode{
    public:
        MoveBaseAction(const std::string& name,const BT::NodeConfig& config) : BT::StatefulActionNode(name,config){}
        static BT::PortsList providedPorts(){
            return {BT::InputPort<Pose2D>("goal")};
        }
        BT::NodeStatus onStart() override{
            if(!getInput<Pose2D>("goal",_goal)){
                throw BT::RuntimeError("missing required input [goal]");
            }
            printf("[MoveBase: SEND REQUEST ]. goal: x=%.2f y=%.2f theta=%.2f\n",
                _goal.x,_goal.y,_goal.theta);
            // simulate action duration of 220 milliseconds
            _completion_time = chr::system_clock::now() + chr::milliseconds(220);
            return BT::NodeStatus::RUNNING;
        }
        BT::NodeStatus onRunning() override{
            // simulate periodic check with a 10 ms delay
            std::this_thread::sleep_for(chr::milliseconds(10));
            // check if the simulated action duration has elapsed
            if (chr::system_clock::now() >= _completion_time){
                printf("[MoveBase completed]\n");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        void onHalted() override{
            printf("[MoveBase: HALTED]");
        }
    private:
        Pose2D _goal;
        chr::system_clock::time_point _completion_time;
};

int main(){
    BT::BehaviorTreeFactory factory;
    auto package_share_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");

    factory.registerNodeType<MoveBaseAction>("MoveBaseAction");
    // Create a Blackboard
    // BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

    // // Set the goal on the Blackboard
    // Pose2D destination;
    // destination.x = 1.0;  // Example values
    // destination.y = 2.0;
    // destination.theta = 0.5;
    // blackboard->set("Destination", destination); // "Destination" matches XML

    auto tree = factory.createTreeFromFile(package_share_path+"/bt_xml/reactive_sequence.xml");

    tree.tickWhileRunning();

    return 0;
}