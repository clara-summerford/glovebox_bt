#include "glovebox_bt/behaviors/move_arm_to_pose.hpp"

#include <numbers>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MoveArmToPose::MoveArmToPose(
    const std::string& name, 
    const BT::NodeConfig& config,
    std::weak_ptr<rclcpp::Node> node_ptr,
    tf2_ros::Buffer::SharedPtr tf_buffer
) :
    
BT::StatefulActionNode(name, config),

// TF setup, member variable (buffer and node) initialization
tf_buffer_(tf_buffer),
node_ptr_(node_ptr),
move_group_(node, "arm")
{    
}

// Constructor setup
// {
//     if (!tf_buffer_) {
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
//         tf_listener_.emplace(*tf_buffer_);
//     }
// }

BT::PortsList MoveArmToPose::providedPorts() {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("target_pose"),
            // BT::InputPort<float>("planning_timeout", 5.0, "The time to wait for the planner to compute in seconds. Default 5 seconds"),
            // BT::InputPort<float>("max_velocity_scaling_factor", 1.0f, "Ratio of maximum allowed joint velocities to those specified in the robot config"),
            // BT::InputPort<float>("max_acceleration_scaling_factor", 1.0f, "Ratio of maximum allowed joint accelerations to those specified in the robot config")
        };
    }

BT::NodeStatus MoveArmToPose::onStart() {

    // need to lock weak pointers every time you want to use them
    rclcpp::Node::SharedPtr node = node_ptr_.lock();

    // Checking if a valid pose was retrieved from blackboard
    auto target_pose = getInput<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose");


    if (!target_pose)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "MoveArmToPose: missing input port [target_pose]: %s",
                     target_pose.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // maybe remove (*), used to separate pose value from pointer
    // or change how target_pose is instantiated (not as a pointer?) 
    geometry_msgs::msg::PoseStamped pose = *target_pose.value();
    RCLCPP_INFO(node->get_logger(), "target pose (in frame %s) recieved: x=%.3f y=%.3f z=%.3f",
                pose.header.frame_id.c_str(),
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z);


    // MoveIt setup
    // moveit::planning_interface::MoveGroupInterface::Options move_group_options(
    //     "arm", "robot_description", "/spot_moveit");
    // moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm", "robot_description");
    // move_group_ = std::<moveit::planning_interface::MoveGroupInterface>(node, move_group_options);

    // Adding moveit configs 
    // retrievePortValueNamed(move_group_->planningTime(), "planning_timeout");
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.2);
    move_group_.setMaxAccelerationScalingFactor(0.2);
    // move_group_.pipeline_id = "ompl";

    // Creating pose goal
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_.setPoseTarget(pose);
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_.execute(plan);
        RCLCPP_INFO(node->get_logger(), "Moved to pick pose.");
        } else {
            RCLCPP_WARN(node->get_logger(), "Planning failed at pick pose.");
            return BT::NodeStatus::FAILURE;
        }

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus MoveHandToPose::onRunning() {
    rclcpp::Node::SharedPtr node = node_ptr_.lock();

    if (!node) { // if node doesn't exist for some reason
        onHalted();
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(node->get_logger(), "Behavior is in an undefined state! Reporting failure!");
    return BT::NodeStatus::FAILURE;
}

void MoveArmToPose::onHalted() {        
    rclcpp::Node::SharedPtr node = node_ptr_.lock();
    RCLCPP_WARN(node->get_logger(), "MoveArmToPose halted");
}