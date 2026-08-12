#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    // ros2 initialization
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() 
    { 
        executor.spin();
    });
    // moveit initialization
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    // set the target pose for the end effector
    arm.setStartStateToCurrentState();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.4;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    arm.setPoseTarget(target_pose, "wrist_link_3");

    moveit::planning_interface::MoveGroupInterface::Plan plan_1;

    bool success = static_cast<bool>(arm.plan(plan_1));
    if (success)
    {
        arm.execute(plan_1);
        arm.clearPoseTargets();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed");
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}