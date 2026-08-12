#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



int main(int argc, char **argv)
{
    // ros2 initialization
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>(
    "moveit_node",
    rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() 
    { 
        executor.spin();
    });
    // moveit initialization
    shape_msgs::msg::SolidPrimitive desk_shape;
    desk_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    desk_shape.dimensions = {0.8, 0.6, 0.05};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject desk;
    desk.id = "desk";
    desk.header.frame_id = "world";
    
    geometry_msgs::msg::Pose desk_pose;
    desk_pose.orientation.w = 1.0;
    desk_pose.position.x = 0.0;
    desk_pose.position.y = 0.0;
    desk_pose.position.z = 0.0;
    
    desk.primitives.push_back(desk_shape);
    desk.primitive_poses.push_back(desk_pose);
    desk.operation = desk.ADD;
    
    planning_scene_interface.applyCollisionObject(desk);
    
    shape_msgs::msg::SolidPrimitive pallet_shape;
    pallet_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    pallet_shape.dimensions = {0.20, 0.18, 0.034};

    moveit_msgs::msg::CollisionObject pallet;
    pallet.id = "pallet";
    pallet.header.frame_id = "place_1";

    geometry_msgs::msg::Pose pallet_pose;
    pallet_pose.position.x = 0.0;
    pallet_pose.position.y = 0.0;
    pallet_pose.position.z = 0.017;
    pallet_pose.orientation.w = 1.0;

    pallet.primitives.push_back(pallet_shape);
    pallet.primitive_poses.push_back(pallet_pose);
    pallet.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(pallet);

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    // set the target pose for the end effector

    arm.setPoseReferenceFrame("base_link");
    arm.setEndEffectorLink("wrist_link_3");
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = 0.4;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.4;

    // target_pose.orientation.x = 0.0;
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 1.0;

    // arm.setPoseTarget(target_pose, "wrist_link_3");

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