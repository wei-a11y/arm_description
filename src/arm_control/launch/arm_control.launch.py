from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("ust")

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
            Node(
                package="arm_control",
                executable="arm_control",
                name="moveit_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        )
                    },
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )