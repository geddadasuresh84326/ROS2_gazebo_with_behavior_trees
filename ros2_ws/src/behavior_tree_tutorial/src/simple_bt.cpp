#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

class SayHello : public BT::SyncActionNode {
public:
    SayHello(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("BT_NODE"), "Hello from Behavior Tree");
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto package_share_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SayHello>("SayHello");

    auto tree = factory.createTreeFromFile(package_share_path + "/bt_xml/simple_bt.xml");

    tree.tickWhileRunning();  // Execute the behavior tree

    rclcpp::shutdown();
    return 0;
}
