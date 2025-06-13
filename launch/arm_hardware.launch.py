from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="turtlebot3_arm.urdf.xacro",
            description="URDF/XACRO description file for the TurtleBot3 arm.",
        ),
    ]

    prefix = LaunchConfiguration("prefix")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare("turtlebot3_autorace_arm"), "config", description_file]),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_autorace_arm"), "config", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[robot_description],
        output="both",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        parameters=[robot_description],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            robot_state_publisher_node,
        ]
    )
