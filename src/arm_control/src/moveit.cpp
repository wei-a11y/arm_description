#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool loadMeshMessage(const std::string& resource, shape_msgs::msg::Mesh& mesh_message)
{
    shapes::ShapePtr mesh_shape
    (
        shapes::createMeshFromResource(resource)
    );

    if (!mesh_shape)
    {
        return false;
    }

    shapes::ShapeMsg shape_message;

    if (!shapes::constructMsgFromShape(
            mesh_shape.get(),
            shape_message))
    {
        return false;
    }

    auto* converted_mesh = boost::get<shape_msgs::msg::Mesh>(&shape_message);

    if (converted_mesh == nullptr)
    {
        return false;
    }

    mesh_message = *converted_mesh;
    return true;
}

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
    //使用stl添加desk模型到规划场景中
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::CollisionObject desk;
    desk.id = "desk";
    desk.header.frame_id = "world";

    shape_msgs::msg::Mesh desk_mesh;

    if (!loadMeshMessage(
            "package://arm_description/meshes/desk/desk_link.STL",
            desk_mesh))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to load desk STL"
        );

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    geometry_msgs::msg::Pose desk_pose;
    desk_pose.orientation.x = 0.0;
    desk_pose.orientation.y = 0.0;
    desk_pose.orientation.z = 0.0;
    desk_pose.orientation.w = 1.0;

    desk_pose.position.x = 0.0;
    desk_pose.position.y = 0.0;
    desk_pose.position.z = 0.20;

    desk.meshes.push_back(desk_mesh);
    desk.mesh_poses.push_back(desk_pose);
    desk.operation = moveit_msgs::msg::CollisionObject::ADD;
    std_msgs::msg::ColorRGBA desk_color;
    desk_color.r = 0.79;
    desk_color.g = 0.81;
    desk_color.b = 0.93;
    desk_color.a = 1.0;

    if (!planning_scene_interface.applyCollisionObject(desk,desk_color))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to add desk to planning scene"
        );
    }
    //
    //使用stl添加pallet模型到规划场景中
    moveit_msgs::msg::CollisionObject pallet;
    pallet.id = "pallet";
    pallet.header.frame_id = "place_1";

    shape_msgs::msg::Mesh pallet_mesh;

    if (!loadMeshMessage(
            "package://arm_description/meshes/pallet/pallet_link.STL",
            pallet_mesh))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to load pallet STL"
        );

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }
    geometry_msgs::msg::Pose pallet_pose;
    pallet_pose.orientation.x = 0.0;
    pallet_pose.orientation.y = 0.0;
    pallet_pose.orientation.z = 0.0;
    pallet_pose.orientation.w = 1.0;
    
    pallet_pose.position.x = 0.0;
    pallet_pose.position.y = 0.0;
    pallet_pose.position.z = 0.017;
    
    pallet.meshes.push_back(pallet_mesh);
    pallet.mesh_poses.push_back(pallet_pose);
    pallet.operation = moveit_msgs::msg::CollisionObject::ADD;
    //添加模型颜色

    std_msgs::msg::ColorRGBA pallet_color;
    pallet_color.r = 0.5;
    pallet_color.g = 0.5;
    pallet_color.b = 0.5;
    pallet_color.a = 1.0;

    if (!planning_scene_interface.applyCollisionObject(pallet, pallet_color))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to add pallet to planning scene"
        );
    }

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    // set the target pose for the end effector

    arm.setPoseReferenceFrame("base_link");
    arm.setEndEffectorLink("wrist_link_3");
    arm.setStartStateToCurrentState();

    // 读取末端当前位姿，保持当前位置不变
    geometry_msgs::msg::Pose target_pose =
        arm.getCurrentPose("wrist_link_3").pose;

    tf2::Quaternion current_orientation;
    tf2::fromMsg(
        target_pose.orientation,
        current_orientation
    );

    
    // 相对于当前姿态绕 Y 轴旋转 0.1 rad
    tf2::Quaternion delta_orientation;
    delta_orientation.setRPY(0.0, 0.0, 0.0);

    tf2::Quaternion target_orientation =
        current_orientation * delta_orientation;

    target_orientation.normalize();
    // target_pose.orientation = tf2::toMsg(target_orientation);
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.position.x -= 0.0;
    target_pose.position.y += 0.0;
    target_pose.position.z -= 0.0;
    bool target_success =
        arm.setPoseTarget(target_pose, "wrist_link_3");

    if (!target_success)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Target pose is invalid"
        );

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

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