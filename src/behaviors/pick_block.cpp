#include "glovebox_BT/behaviors/pick_block.hpp"

PickBlock::PickBlock(
    const std::string& name, 
    const BT::NodeConfig& config)

    : BT::StatefulActionNode(name, config)

// Constructor setup
{
    try {
    node_ = rclcpp::Node::make_shared("drop_ball");

    // Get robot_description from move_group's namespace
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        node_, "/spot_moveit/move_group");
    
    // Wait for the parameter service to be available
    if (!param_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), 
            "Parameter service /move_group not available");
        throw std::runtime_error("move_group parameter service not available");
    }
    
    // MoveIt setup
    moveit::planning_interface::MoveGroupInterface::Options move_group_options(
        "arm", "robot_description", "/spot_moveit");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, move_group_options);

    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    catch (const std::exception& e) {
        std::cerr << "DropBall constructor EXCEPTION: " << e.what() << std::endl;
        throw;  // rethrow so BT can fail clearly
    }
    
}

BT::PortsList PickBlock::providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("block_pose") 
        };
    }

BT::OnStart()