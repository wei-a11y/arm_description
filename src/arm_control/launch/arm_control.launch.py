from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "arm_description",
            package_name="moveit_config",
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="arm_control",
                executable="arm_control",
                name="moveit_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )