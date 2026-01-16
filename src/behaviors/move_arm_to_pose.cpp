#include "glovebox_bt/behaviors/move_arm_to_pose.hpp"

MoveArmToPose::MoveArmToPose(
    const std::string& name, 
    const BT::NodeConfig& config,
    std::weak_ptr<rclcpp::Node> node_ptr,
    tf2_ros::Buffer::SharedPtr tf_buffer
    ) :
    
    BT::StatefulActionNode(name, config),
    // TF setup, member variable (tf_buffer_) initialization

    tf_buffer_(tf_buffer)

// Constructor setup
{
    if (!tf_buffer_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
        tf_listener_.emplace(*tf_buffer_);
    }
}

BT::PortsList MoveArmToPose::providedPorts() {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("target_pose"),
            // BT::InputPort<float>("planning_timeout", 5.0, "The time to wait for the planner to compute in seconds. Default 5 seconds"),
            // BT::InputPort<float>("max_velocity_scaling_factor", 1.0f, "Ratio of maximum allowed joint velocities to those specified in the robot config"),
            // BT::InputPort<float>("max_acceleration_scaling_factor", 1.0f, "Ratio of maximum allowed joint accelerations to those specified in the robot config")
        };
    }

BT::NodeStatus MoveArmToPose::onStart() {

    rclcpp::Node::SharedPtr node = get_node();

    // Checking if a valid pose was retrieved from blackboard
    // auto pose = getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("target_pose");
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose = retrievePortValue(target_pose);

    if (!target_pose)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "MoveArmToPose: missing input port [target_pose]: %s",
                     pose.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // maybe remove (*), used to separate pose value from pointer
    // or change how target_pose is instantiated (not as a pointer?) 
    // geometry_msgs::msg::PoseStamped target_pose = *pose.value();
    RCLCPP_INFO(node_->get_logger(), "target pose (in frame %s) recieved: x=%.3f y=%.3f z=%.3f",
                target_pose.header.frame_id,
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);


    // MoveIt setup
    // moveit::planning_interface::MoveGroupInterface::Options move_group_options(
    //     "arm", "robot_description", "/spot_moveit");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm", "robot_description");

    // Adding moveit configs 
    // retrievePortValueNamed(move_group_->planningTime(), "planning_timeout");
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    // move_group_.pipeline_id = "ompl";

    // Creating pose goal
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_.setPoseTarget(target_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(node_->get_logger(), "Moved to pick pose.");
        } else {
            RCLCPP_WARN(node_->get_logger(), "Planning failed at pick pose.");
            return 1;
        }

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus MoveHandToPose::onRunning() {
    if (!get_node()) { // if node doesn't exist for some reason
        onHalted();
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(node_->get_logger(), "Behavior is in an undefined state! Reporting failure!");
    return BT::NodeStatus::FAILURE;
}

void MoveArmToPose::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "MoveArmToPose halted");
}