#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/variant/get.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
    sync_pallet_from_gazebo_ =
      get_or_declare_parameter<bool>("sync_pallet_from_gazebo", false);
    gazebo_pallet_link_name_ = get_or_declare_parameter<std::string>(
      "gazebo_pallet_link_name", "pallet::pallet_link");
    pallet_object_id_ = get_or_declare_parameter<std::string>("pallet_object_id", "pallet");
    pallet_sync_rate_ = get_or_declare_parameter<double>("pallet_sync_rate", 10.0);
    pallet_touch_links_ = get_or_declare_parameter<std::vector<std::string>>(
      "pallet_touch_links", {"fork_link_L", "fork_link_R", "wrist_link_3"});

    velocity_scaling_ = clamp_scaling_factor(velocity_scaling_, "max_velocity_scaling_factor");
    acceleration_scaling_ =
      clamp_scaling_factor(acceleration_scaling_, "max_acceleration_scaling_factor");
    if (pallet_sync_rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "pallet_sync_rate must be positive; using 10.0 Hz");
      pallet_sync_rate_ = 10.0;
    }
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

      attach_pallet_service_ = create_service<std_srvs::srv::Trigger>(
        "~/attach_pallet",
        std::bind(
          &ArmControlNode::attach_pallet_callback, this, std::placeholders::_1,
          std::placeholders::_2),
        rmw_qos_profile_services_default, control_callback_group_);
      detach_pallet_service_ = create_service<std_srvs::srv::Trigger>(
        "~/detach_pallet",
        std::bind(
          &ArmControlNode::detach_pallet_callback, this, std::placeholders::_1,
          std::placeholders::_2),
        rmw_qos_profile_services_default, control_callback_group_);

      if (sync_pallet_from_gazebo_) {
        pallet_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pallet_state_callback_group_ =
          create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions pallet_subscription_options;
        pallet_subscription_options.callback_group = pallet_state_callback_group_;
        pallet_link_states_subscription_ = create_subscription<gazebo_msgs::msg::LinkStates>(
          "/gazebo/link_states", rclcpp::SensorDataQoS(),
          std::bind(&ArmControlNode::link_states_callback, this, std::placeholders::_1),
          pallet_subscription_options);

        pallet_scene_callback_group_ =
          create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const auto sync_period = std::chrono::duration<double>(1.0 / pallet_sync_rate_);
        pallet_scene_timer_ = create_wall_timer(
          sync_period, std::bind(&ArmControlNode::sync_pallet_planning_scene, this),
          pallet_scene_callback_group_);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize arm control: %s", error.what());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Arm control is ready. Publish geometry_msgs/msg/PoseStamped commands to '%s'",
      target_pose_subscription_->get_topic_name());
    if (sync_pallet_from_gazebo_) {
      RCLCPP_INFO(
        get_logger(), "Synchronizing Gazebo link '%s' with MoveIt object '%s'",
        gazebo_pallet_link_name_.c_str(), pallet_object_id_.c_str());
    }
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

    if (sync_pallet_from_gazebo_) {
      return true;
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
      pallet_object_id_, "place_1", "package://arm_description/meshes/pallet/pallet_link.STL",
      pallet_pose, pallet_color);
  }

  static std_msgs::msg::ColorRGBA pallet_color()
  {
    std_msgs::msg::ColorRGBA color;
    color.r = 0.5F;
    color.g = 0.5F;
    color.b = 0.5F;
    color.a = 1.0F;
    return color;
  }

  static double pose_translation_distance(
    const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
  {
    const double dx = lhs.position.x - rhs.position.x;
    const double dy = lhs.position.y - rhs.position.y;
    const double dz = lhs.position.z - rhs.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static double pose_rotation_distance(
    const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
  {
    const double dot =
      lhs.orientation.x * rhs.orientation.x + lhs.orientation.y * rhs.orientation.y +
      lhs.orientation.z * rhs.orientation.z + lhs.orientation.w * rhs.orientation.w;
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  }

  void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr message)
  {
    const auto link_iterator =
      std::find(message->name.begin(), message->name.end(), gazebo_pallet_link_name_);
    if (link_iterator == message->name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Gazebo link '%s' is not available",
        gazebo_pallet_link_name_.c_str());
      return;
    }

    const auto link_index = static_cast<std::size_t>(
      std::distance(message->name.begin(), link_iterator));
    if (link_index >= message->pose.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Malformed /gazebo/link_states message");
      return;
    }

    geometry_msgs::msg::Pose pallet_pose = message->pose[link_index];
    if (!normalize_and_validate_pose(pallet_pose)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Gazebo returned an invalid pallet pose");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      latest_pallet_pose_ = pallet_pose;
      pallet_pose_received_ = true;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "pallet_link";
    transform.transform.translation.x = pallet_pose.position.x;
    transform.transform.translation.y = pallet_pose.position.y;
    transform.transform.translation.z = pallet_pose.position.z;
    transform.transform.rotation = pallet_pose.orientation;
    pallet_tf_broadcaster_->sendTransform(transform);
  }

  bool normalize_and_validate_pose(geometry_msgs::msg::Pose & pose) const
  {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.pose = pose;
    if (!normalize_and_validate_pose(stamped_pose)) {
      return false;
    }
    pose = stamped_pose.pose;
    return true;
  }

  void sync_pallet_planning_scene()
  {
    if (!add_scene_objects_) {
      return;
    }

    geometry_msgs::msg::Pose current_pose;
    bool scene_initialized = false;
    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (!pallet_pose_received_ || pallet_attached_) {
        return;
      }
      current_pose = latest_pallet_pose_;
      scene_initialized = pallet_scene_initialized_;
      if (
        scene_initialized && last_scene_pose_valid_ &&
        pose_translation_distance(current_pose, last_scene_pose_) <= 0.0005 &&
        pose_rotation_distance(current_pose, last_scene_pose_) <= 0.001745329252)
      {
        return;
      }
    }

    std::lock_guard<std::mutex> scene_lock(scene_operation_mutex_);
    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (pallet_attached_) {
        return;
      }
    }

    bool success = false;
    if (!scene_initialized) {
      success = add_mesh_collision_object(
        pallet_object_id_, "world", "package://arm_description/meshes/pallet/pallet_link.STL",
        current_pose, pallet_color());
    } else {
      moveit_msgs::msg::CollisionObject pallet_move;
      pallet_move.header.frame_id = "world";
      pallet_move.id = pallet_object_id_;
      pallet_move.mesh_poses.push_back(current_pose);
      pallet_move.operation = moveit_msgs::msg::CollisionObject::MOVE;
      success = planning_scene_interface_->applyCollisionObject(pallet_move);
    }

    if (!success) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to synchronize pallet with MoveIt");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      pallet_scene_initialized_ = true;
      last_scene_pose_ = current_pose;
      last_scene_pose_valid_ = true;
    }
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

  bool wait_for_pallet_attached_state(bool expected_attached)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    do {
      const bool is_attached =
        !planning_scene_interface_->getAttachedObjects({pallet_object_id_}).empty();
      if (is_attached == expected_attached) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void attach_pallet_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    bool expected = false;
    if (!command_in_progress_.compare_exchange_strong(expected, true)) {
      response->message = "Arm control is busy";
      return;
    }
    struct CommandGuard
    {
      explicit CommandGuard(std::atomic_bool & busy_flag)
      : busy_flag_(busy_flag) {}
      ~CommandGuard() {busy_flag_.store(false);}
      std::atomic_bool & busy_flag_;
    } command_guard(command_in_progress_);

    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (!sync_pallet_from_gazebo_) {
        response->message = "Gazebo pallet synchronization is disabled";
        return;
      }
      if (!add_scene_objects_) {
        response->message = "Planning Scene objects are disabled";
        return;
      }
      if (!pallet_pose_received_ || !pallet_scene_initialized_) {
        response->message = "Pallet pose has not been synchronized yet";
        return;
      }
      if (pallet_attached_) {
        response->message = "Pallet is already attached in MoveIt";
        return;
      }
    }

    std::lock_guard<std::mutex> scene_lock(scene_operation_mutex_);
    if (!move_group_->attachObject(
        pallet_object_id_, end_effector_link_, pallet_touch_links_))
    {
      response->message = "MoveIt rejected the pallet attach request";
      return;
    }
    if (!wait_for_pallet_attached_state(true)) {
      response->message = "Timed out waiting for MoveIt to attach the pallet";
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      pallet_attached_ = true;
    }
    response->success = true;
    response->message = "Pallet attached in MoveIt; Gazebo remains contact-driven";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void detach_pallet_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    bool expected = false;
    if (!command_in_progress_.compare_exchange_strong(expected, true)) {
      response->message = "Arm control is busy";
      return;
    }
    struct CommandGuard
    {
      explicit CommandGuard(std::atomic_bool & busy_flag)
      : busy_flag_(busy_flag) {}
      ~CommandGuard() {busy_flag_.store(false);}
      std::atomic_bool & busy_flag_;
    } command_guard(command_in_progress_);

    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (!pallet_attached_) {
        response->message = "Pallet is not attached in MoveIt";
        return;
      }
    }

    {
      std::lock_guard<std::mutex> scene_lock(scene_operation_mutex_);
      if (!move_group_->detachObject(pallet_object_id_)) {
        response->message = "MoveIt rejected the pallet detach request";
        return;
      }
      if (!wait_for_pallet_attached_state(false)) {
        response->message = "Timed out waiting for MoveIt to detach the pallet";
        return;
      }

      std::lock_guard<std::mutex> lock(pallet_mutex_);
      pallet_attached_ = false;
      pallet_scene_initialized_ = false;
      last_scene_pose_valid_ = false;
    }

    sync_pallet_planning_scene();
    {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (!pallet_scene_initialized_) {
        response->message = "Pallet detached, but its world pose could not be restored";
        return;
      }
    }

    response->success = true;
    response->message = "Pallet detached and restored from its Gazebo pose";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (sync_pallet_from_gazebo_ && add_scene_objects_) {
      std::lock_guard<std::mutex> lock(pallet_mutex_);
      if (!pallet_pose_received_ || !pallet_scene_initialized_) {
        RCLCPP_WARN(get_logger(), "Rejecting motion until the pallet is synchronized");
        publish_result(false);
        return;
      }
    }

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
  std::string gazebo_pallet_link_name_;
  std::string pallet_object_id_;
  std::vector<std::string> pallet_touch_links_;
  double velocity_scaling_{1.0};
  double acceleration_scaling_{1.0};
  double pallet_sync_rate_{10.0};
  bool add_scene_objects_{true};
  bool sync_pallet_from_gazebo_{false};
  std::atomic_bool command_in_progress_{false};

  std::mutex pallet_mutex_;
  std::mutex scene_operation_mutex_;
  geometry_msgs::msg::Pose latest_pallet_pose_;
  geometry_msgs::msg::Pose last_scene_pose_;
  bool pallet_pose_received_{false};
  bool pallet_scene_initialized_{false};
  bool last_scene_pose_valid_{false};
  bool pallet_attached_{false};

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> pallet_tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pallet_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pallet_scene_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr pallet_link_states_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_pallet_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_pallet_service_;
  rclcpp::TimerBase::SharedPtr pallet_scene_timer_;
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
