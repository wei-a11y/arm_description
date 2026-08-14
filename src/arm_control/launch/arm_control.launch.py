from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    planning_group = LaunchConfiguration("planning_group")
    pose_reference_frame = LaunchConfiguration("pose_reference_frame")
    end_effector_link = LaunchConfiguration("end_effector_link")
    velocity_scaling = LaunchConfiguration("max_velocity_scaling_factor")
    acceleration_scaling = LaunchConfiguration("max_acceleration_scaling_factor")
    add_scene_objects = LaunchConfiguration("add_scene_objects")
    sync_pallet_from_gazebo = LaunchConfiguration("sync_pallet_from_gazebo")
    gazebo_pallet_link_name = LaunchConfiguration("gazebo_pallet_link_name")
    pallet_object_id = LaunchConfiguration("pallet_object_id")
    pallet_sync_rate = LaunchConfiguration("pallet_sync_rate")

    moveit_config = (
        MoveItConfigsBuilder(
            "arm_description",
            package_name="moveit_config",
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="~/target_pose",
                description="PoseStamped command topic",
            ),
            DeclareLaunchArgument("planning_group", default_value="arm"),
            DeclareLaunchArgument("pose_reference_frame", default_value="base_link"),
            DeclareLaunchArgument("end_effector_link", default_value="wrist_link_3"),
            DeclareLaunchArgument(
                "max_velocity_scaling_factor", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "max_acceleration_scaling_factor", default_value="1.0"
            ),
            DeclareLaunchArgument("add_scene_objects", default_value="true"),
            DeclareLaunchArgument(
                "sync_pallet_from_gazebo",
                default_value="false",
                description="Synchronize the pallet pose from Gazebo link states",
            ),
            DeclareLaunchArgument(
                "gazebo_pallet_link_name",
                default_value="pallet::pallet_link",
            ),
            DeclareLaunchArgument("pallet_object_id", default_value="pallet"),
            DeclareLaunchArgument("pallet_sync_rate", default_value="10.0"),
            Node(
                package="arm_control",
                executable="arm_control",
                name="arm_control",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                        "target_pose_topic": target_pose_topic,
                        "planning_group": planning_group,
                        "pose_reference_frame": pose_reference_frame,
                        "end_effector_link": end_effector_link,
                        "max_velocity_scaling_factor": ParameterValue(
                            velocity_scaling,
                            value_type=float,
                        ),
                        "max_acceleration_scaling_factor": ParameterValue(
                            acceleration_scaling,
                            value_type=float,
                        ),
                        "add_scene_objects": ParameterValue(
                            add_scene_objects,
                            value_type=bool,
                        ),
                        "sync_pallet_from_gazebo": ParameterValue(
                            sync_pallet_from_gazebo,
                            value_type=bool,
                        ),
                        "gazebo_pallet_link_name": gazebo_pallet_link_name,
                        "pallet_object_id": pallet_object_id,
                        "pallet_sync_rate": ParameterValue(
                            pallet_sync_rate,
                            value_type=float,
                        ),
                    },
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
