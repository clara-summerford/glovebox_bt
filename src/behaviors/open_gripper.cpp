#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp> 
#include <moveit/move_group_interface/move_group_interface.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gbox_BT_moveit");

OpenGripper::OpenGripper(
    const std::string& name, 
    const BT::NodeConfig& config
    ) : 

StatefulActionNode(name, config)

BT::NodeStatus OpenGripper::onStart() {
    rclcpp::Node::SharedPtr node = get_node();

    // Create the action client, taken from NRG behaviors (maybe avoid namespaces completely)
    // std::string moveit_namespace = retrievePortValue(moveit_namespace);
    const std::string motion_action_name = resolveNamespace(moveit_namespace) + "/move_action";
    execution_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, motion_action_name);

}