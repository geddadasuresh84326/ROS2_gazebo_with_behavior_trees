#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std;

class SaySomething : public BT::SyncActionNode
{
    public:
        SaySomething(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name,config){}
    
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("message")};
        }
        BT::NodeStatus tick() override
        {
            BT::Expected<std::string> msg = getInput<std::string>("message");
            if(!msg){
                throw BT::RuntimeError("missing required input [message] : ",msg.error()); 
            }
            std::cout<<"Robot says:"<<msg.value() <<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

class ThinkWhatToSay : public BT::SyncActionNode{
    public:
        ThinkWhatToSay(const std::string& name,const BT::NodeConfig& config) : SyncActionNode(name,config){}

        static BT::PortsList providedPorts(){
            return {BT::OutputPort<std::string>("text")};
        }       
        BT::NodeStatus tick() override{
            setOutput("text","hi surya");
            return BT::NodeStatus::SUCCESS;
        }
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    auto package_share_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");
    auto tree = factory.createTreeFromFile(package_share_path + "/bt_xml/black_board_ports.xml");
    tree.tickWhileRunning();
    rclcpp::shutdown();
    return 0;   
};