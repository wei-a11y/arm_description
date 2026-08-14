import os
from pathlib import Path
from xml.dom import Node as DomNode

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _prepend_environment_path(name, path):
    """Make Gazebo's system resources available in a clean ROS shell."""
    current_paths = [item for item in os.environ.get(name, "").split(":") if item]
    if path not in current_paths:
        os.environ[name] = ":".join([path, *current_paths])


def _compact_xacro(path, mappings=None):
    """Expand Xacro and remove comments unsafe for ROS CLI overrides."""
    document = xacro.process_file(str(path), mappings=mappings or {})

    def remove_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == DomNode.COMMENT_NODE:
                node.removeChild(child)
            else:
                remove_comments(child)

    remove_comments(document)
    return document.toxml()


def generate_launch_description():
    _prepend_environment_path(
        "GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"
    )
    _prepend_environment_path(
        "GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"
    )
    _prepend_environment_path(
        "GAZEBO_PLUGIN_PATH", "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
    )
    os.environ.setdefault(
        "OGRE_RESOURCE_PATH", "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"
    )

    arm_description_share = Path(
        get_package_share_directory("arm_description")
    )
    _prepend_environment_path(
        "GAZEBO_MODEL_PATH", str(arm_description_share.parent)
    )
    os.environ.setdefault("GAZEBO_MODEL_DATABASE_URI", "")
    robot_bringup_share = Path(
        get_package_share_directory("robot_bringup")
    )
    moveit_config_share = Path(
        get_package_share_directory("moveit_config")
    )
    arm_control_share = Path(
        get_package_share_directory("arm_control")
    )
    gazebo_ros_share = Path(
        get_package_share_directory("gazebo_ros")
    )

    arm_xacro = arm_description_share / "urdf" / "arm.urdf.xacro"
    desk_xacro = (
        arm_description_share / "urdf" / "desk.gazebo.urdf.xacro"
    )
    pallet_xacro = (
        arm_description_share / "urdf" / "pallet.gazebo.urdf.xacro"
    )
    rviz_config = robot_bringup_share / "rviz" / "moveit.rviz"
    world_file = robot_bringup_share / "worlds" / "arm.world"

    # toxml() produces compact XML. This is required by the Humble
    # gazebo_ros2_control plugin when it forwards robot_description to its
    # internally-created controller_manager.
    arm_description = ParameterValue(
        _compact_xacro(arm_xacro, mappings={"use_gazebo": "true"}),
        value_type=str,
    )
    desk_description = ParameterValue(
        _compact_xacro(desk_xacro),
        value_type=str,
    )
    pallet_description = ParameterValue(
        _compact_xacro(pallet_xacro),
        value_type=str,
    )

    start_rviz = LaunchConfiguration("start_rviz")
    start_arm_control = LaunchConfiguration("start_arm_control")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gazebo.launch.py")
        ),
        launch_arguments={"world": str(world_file)}.items(),
    )

    # The arm description is also the source used by spawn_entity.py.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": arm_description}],
    )

    # Separate description topics let Gazebo create the planning-scene
    # obstacles as independent entities.
    desk_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="desk_state_publisher",
        output="screen",
        parameters=[{"robot_description": desk_description}],
        remappings=[("robot_description", "desk_description")],
    )
    pallet_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="pallet_state_publisher",
        output="screen",
        parameters=[{"robot_description": pallet_description}],
        remappings=[("robot_description", "pallet_description")],
    )

    spawn_arm = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_arm",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "arm",
        ],
    )
    spawn_desk = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_desk",
        output="screen",
        arguments=[
            "-topic", "desk_description",
            "-entity", "desk",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.20",
        ],
    )
    spawn_pallet = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_pallet",
        output="screen",
        arguments=[
            "-topic", "pallet_description",
            "-entity", "pallet",
            "-x", "0.2294165545",
            "-y", "0.19",
            "-z", "0.695",
            "-Y", "1.5707963268",
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_arm_controller",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config_share / "launch" / "move_group.launch.py")
        )
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(start_rviz),
    )

    # arm_control adds desk and pallet to MoveIt's Planning Scene, then runs
    # the example motion. It is optional because users may only want bringup.
    arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_control_share / "launch" / "arm_control.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "sync_pallet_from_gazebo": "true",
        }.items(),
        condition=IfCondition(start_arm_control),
    )

    world_to_desk = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_desk_link",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.20",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "world", "--child-frame-id", "desk_link",
        ],
    )
    world_to_place_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_place_1",
        arguments=[
            "--x", "0.2294165545",
            "--y", "0.19",
            "--z", "0.678",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "1.5707963268",
            "--frame-id", "world", "--child-frame-id", "place_1",
        ],
    )

    # Gazebo's spawn service handles one insertion at a time. Sequence all
    # entities to avoid concurrent factory requests, then start controllers.
    start_pallet = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_desk,
            on_exit=[spawn_pallet],
        )
    )
    start_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_pallet,
            on_exit=[spawn_arm],
        )
    )
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm,
            on_exit=[joint_state_broadcaster],
        )
    )
    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )
    start_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[
                move_group,
                rviz,
                TimerAction(period=3.0, actions=[arm_control]),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "start_arm_control",
                default_value="false",
                description=(
                    "Add the Planning Scene objects and run the example motion."
                ),
            ),
            SetParameter(name="use_sim_time", value=True),
            start_pallet,
            start_arm,
            start_joint_state_broadcaster,
            start_arm_controller,
            start_moveit,
            gazebo,
            robot_state_publisher,
            desk_state_publisher,
            pallet_state_publisher,
            world_to_desk,
            world_to_place_1,
            spawn_desk,
        ]
    )
