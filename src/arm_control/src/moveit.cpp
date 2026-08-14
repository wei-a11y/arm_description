#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace arm_control
{

class ArmControlNode final : public rclcpp::Node
{
public:
  explicit ArmControlNode(const rclcpp::NodeOptions & options)
  : Node("arm_control", options)
  {
    planning_group_ = get_or_declare_parameter<std::string>("planning_group", "arm");
    pose_reference_frame_ =
      get_or_declare_parameter<std::string>("pose_reference_frame", "base_link");
    end_effector_link_ =
      get_or_declare_parameter<std::string>("end_effector_link", "wrist_link_3");
    target_pose_topic_ =
      get_or_declare_parameter<std::string>("target_pose_topic", "~/target_pose");
    velocity_scaling_ =
      get_or_declare_parameter<double>("max_velocity_scaling_factor", 1.0);
    acceleration_scaling_ =
      get_or_declare_parameter<double>("max_acceleration_scaling_factor", 1.0);
    add_scene_objects_ = get_or_declare_parameter<bool>("add_scene_objects", true);

    velocity_scaling_ = clamp_scaling_factor(velocity_scaling_, "max_velocity_scaling_factor");
    acceleration_scaling_ =
      clamp_scaling_factor(acceleration_scaling_, "max_acceleration_scaling_factor");
  }

  bool initialize()
  {
    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      planning_scene_interface_ =
        std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

      move_group_->setPoseReferenceFrame(pose_reference_frame_);
      move_group_->setEndEffectorLink(end_effector_link_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

      if (add_scene_objects_ && !add_collision_objects()) {
        return false;
      }

      result_publisher_ = create_publisher<std_msgs::msg::Bool>("~/execution_success", 10);
      current_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);

      // MoveIt uses callbacks on this node while a planning request is blocking. A reentrant
      // callback group lets a multithreaded executor continue processing those callbacks.
      control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions subscription_options;
      subscription_options.callback_group = control_callback_group_;
      target_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10),
        std::bind(&ArmControlNode::target_pose_callback, this, std::placeholders::_1),
        subscription_options);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize arm control: %s", error.what());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Arm control is ready. Publish geometry_msgs/msg/PoseStamped commands to '%s'",
      target_pose_subscription_->get_topic_name());
    return true;
  }

private:
  template<typename ParameterT>
  ParameterT get_or_declare_parameter(const std::string & name, const ParameterT & default_value)
  {
    if (!has_parameter(name)) {
      return declare_parameter<ParameterT>(name, default_value);
    }

    ParameterT value{};
    get_parameter(name, value);
    return value;
  }

  double clamp_scaling_factor(double value, const char * parameter_name)
  {
    if (value >= 0.0 && value <= 1.0) {
      return value;
    }

    const double clamped_value = std::max(0.0, std::min(1.0, value));
    RCLCPP_WARN(
      get_logger(), "%s must be in [0.0, 1.0]; using %.3f", parameter_name, clamped_value);
    return clamped_value;
  }

  static bool load_mesh_message(
    const std::string & resource, shape_msgs::msg::Mesh & mesh_message)
  {
    const shapes::ShapePtr mesh_shape(shapes::createMeshFromResource(resource));
    if (!mesh_shape) {
      return false;
    }

    shapes::ShapeMsg shape_message;
    if (!shapes::constructMsgFromShape(mesh_shape.get(), shape_message)) {
      return false;
    }

    const auto * converted_mesh = boost::get<shape_msgs::msg::Mesh>(&shape_message);
    if (converted_mesh == nullptr) {
      return false;
    }

    mesh_message = *converted_mesh;
    return true;
  }

  bool add_mesh_collision_object(
    const std::string & id, const std::string & frame_id, const std::string & resource,
    const geometry_msgs::msg::Pose & pose, const std_msgs::msg::ColorRGBA & color)
  {
    shape_msgs::msg::Mesh mesh;
    if (!load_mesh_message(resource, mesh)) {
      RCLCPP_ERROR(get_logger(), "Failed to load mesh '%s'", resource.c_str());
      return false;
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    collision_object.meshes.push_back(std::move(mesh));
    collision_object.mesh_poses.push_back(pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    if (!planning_scene_interface_->applyCollisionObject(collision_object, color)) {
      RCLCPP_ERROR(get_logger(), "Failed to add '%s' to the planning scene", id.c_str());
      return false;
    }
    return true;
  }

  bool add_collision_objects()
  {
    geometry_msgs::msg::Pose desk_pose;
    desk_pose.orientation.w = 1.0;
    desk_pose.position.z = 0.20;

    std_msgs::msg::ColorRGBA desk_color;
    desk_color.r = 0.79F;
    desk_color.g = 0.81F;
    desk_color.b = 0.93F;
    desk_color.a = 1.0F;

    if (!add_mesh_collision_object(
        "desk", "world", "package://arm_description/meshes/desk/desk_link.STL", desk_pose,
        desk_color))
    {
      return false;
    }

    geometry_msgs::msg::Pose pallet_pose;
    pallet_pose.orientation.w = 1.0;
    pallet_pose.position.z = 0.017;

    std_msgs::msg::ColorRGBA pallet_color;
    pallet_color.r = 0.5F;
    pallet_color.g = 0.5F;
    pallet_color.b = 0.5F;
    pallet_color.a = 1.0F;

    return add_mesh_collision_object(
      "pallet", "place_1", "package://arm_description/meshes/pallet/pallet_link.STL", pallet_pose,
      pallet_color);
  }

  bool normalize_and_validate_pose(geometry_msgs::msg::PoseStamped & target) const
  {
    auto & pose = target.pose;
    const bool position_is_finite =
      std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z);
    const bool orientation_is_finite =
      std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
    if (!position_is_finite || !orientation_is_finite) {
      return false;
    }

    const double norm = std::sqrt(
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (norm < 1.0e-6) {
      return false;
    }

    pose.orientation.x /= norm;
    pose.orientation.y /= norm;
    pose.orientation.z /= norm;
    pose.orientation.w /= norm;
    if (target.header.frame_id.empty()) {
      target.header.frame_id = pose_reference_frame_;
    }
    return true;
  }

  void publish_result(bool success)
  {
    std_msgs::msg::Bool result;
    result.data = success;
    result_publisher_->publish(result);
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    bool expected = false;
    if (!command_in_progress_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Arm is busy; rejecting the new target pose");
      publish_result(false);
      return;
    }

    struct CommandGuard
    {
      explicit CommandGuard(std::atomic_bool & busy_flag)
      : busy_flag_(busy_flag) {}
      ~CommandGuard()
      {
        busy_flag_.store(false);
      }
      std::atomic_bool & busy_flag_;
    } command_guard(command_in_progress_);

    geometry_msgs::msg::PoseStamped target = *message;
    if (!normalize_and_validate_pose(target)) {
      RCLCPP_ERROR(get_logger(), "Target pose contains an invalid position or quaternion");
      publish_result(false);
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Received target pose in frame '%s': [%.3f, %.3f, %.3f]",
      target.header.frame_id.c_str(), target.pose.position.x, target.pose.position.y,
      target.pose.position.z);

    bool success = false;
    try {
      move_group_->setStartStateToCurrentState();
      if (!move_group_->setPoseTarget(target, end_effector_link_)) {
        RCLCPP_ERROR(get_logger(), "MoveIt rejected the target pose");
      } else {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!static_cast<bool>(move_group_->plan(plan))) {
          RCLCPP_ERROR(get_logger(), "Motion planning failed");
        } else {
          success = static_cast<bool>(move_group_->execute(plan));
          if (!success) {
            RCLCPP_ERROR(get_logger(), "Trajectory execution failed");
          }
        }
      }
      move_group_->clearPoseTargets();
    } catch (const std::exception & error) {
      move_group_->clearPoseTargets();
      RCLCPP_ERROR(get_logger(), "MoveIt command failed: %s", error.what());
    }

    publish_result(success);
    current_pose_publisher_->publish(move_group_->getCurrentPose(end_effector_link_));
    if (success) {
      RCLCPP_INFO(get_logger(), "Target pose executed successfully");
    }
  }

  std::string planning_group_;
  std::string pose_reference_frame_;
  std::string end_effector_link_;
  std::string target_pose_topic_;
  double velocity_scaling_{1.0};
  double acceleration_scaling_{1.0};
  bool add_scene_objects_{true};
  std::atomic_bool command_in_progress_{false};

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
};

}  // namespace arm_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  const auto node = std::make_shared<arm_control::ArmControlNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  if (!node->initialize()) {
    rclcpp::shutdown();
    executor_thread.join();
    return 1;
  }

  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
