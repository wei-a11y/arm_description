from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_description_share = Path(
        get_package_share_directory("arm_description")
    )
    robot_bringup_share = Path(
        get_package_share_directory("robot_bringup")
    )
    moveit_config_share = Path(
        get_package_share_directory("moveit_config")
    )

    urdf_path = arm_description_share / "urdf" / "arm.urdf.xacro"
    rviz_config_path = robot_bringup_share / "rviz" / "moveit.rviz"
    controllers_config_path = (
        robot_bringup_share / "config" / "ros2_controllers.yaml"
    )
    move_group_launch_path = (
        moveit_config_share / "launch" / "move_group.launch.py"
    )

    robot_description = ParameterValue(
        Command(["xacro ", str(urdf_path)]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    str(controllers_config_path)],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(move_group_launch_path))
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config_path)],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                arguments=[
                    "--x", "-0.25",
                    "--y", "0.19",
                    "--z", "0.075",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "world",
                    "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="place_1_tf",
                arguments=[
                    "--x", "0.2294",
                    "--y", "0.19",
                    "--z", "0.478",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "1.5708",
                    "--frame-id", "world",
                    "--child-frame-id", "place_1",
                ],
            ),
        ]
    )
