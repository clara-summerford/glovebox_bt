// testing
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

// custom nodes
#include "glovebox_bt/behaviors/move_arm_to_pose.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS2 client library
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    // Create the shared node for ALL BT actions
    auto node = rclcpp::Node::make_shared("glovebox_bt");

    BT::BehaviorTreeFactory factory;
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Register custom nodes
    factory.registerNodeType<MoveArmToPose>("MoveArmToPose");

    // Load BehaviorTree from installed file
    std::string share_path = ament_index_cpp::get_package_share_directory("glovebox_bt");
    std::string xml_path = share_path + "/behavior_trees/glovebox_tree.xml";

    // Create the tree
    auto tree = factory.createTreeFromFile(xml_path);

    // hard coding target pose for testing
    auto target_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    target_pose->header.frame_id = "Glovebox"; // check if capital G
    target_pose->pose.position.x = -0.80974;  
    target_pose->pose.position.y = -0.01147;  
    target_pose->pose.position.z = 0.320991;
    tree.rootBlackboard()->set<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("target_pose", target_pose);

    // Tick the tree
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;

}
