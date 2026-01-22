import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controllers_pkg = get_package_share_directory("legged_hunter_controllers")
    description_pkg = get_package_share_directory("legged_hunter_description")

    default_task = os.path.join(controllers_pkg, "config", "ocs2", "task.info")
    default_reference = os.path.join(controllers_pkg, "config", "ocs2", "reference.info")
    default_gait = os.path.join(controllers_pkg, "config", "ocs2", "gait.info")
    default_urdf = os.path.join(description_pkg, "urdf", "hunter.urdf")

    return LaunchDescription([
        DeclareLaunchArgument(
            name="taskFile",
            default_value=default_task,
        ),
        DeclareLaunchArgument(
            name="referenceFile",
            default_value=default_reference,
        ),
        DeclareLaunchArgument(
            name="gaitCommandFile",
            default_value=default_gait,
        ),
        DeclareLaunchArgument(
            name="urdfFile",
            default_value=default_urdf,
        ),
        DeclareLaunchArgument(
            name="enableKeepalive",
            default_value="true",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[LaunchConfiguration("urdfFile")],
        ),
        Node(
            package="ocs2_legged_robot_ros",
            executable="legged_robot_sqp_mpc",
            name="legged_robot_sqp_mpc",
            output="screen",
            parameters=[
                {"taskFile": LaunchConfiguration("taskFile")},
                {"referenceFile": LaunchConfiguration("referenceFile")},
                {"urdfFile": LaunchConfiguration("urdfFile")},
            ],
        ),
        Node(
            package="ocs2_legged_robot_ros",
            executable="legged_robot_dummy",
            name="legged_robot_dummy",
            output="screen",
            remappings=[
                ("joint_states", "legged_robot_dummy/joint_states"),
            ],
            parameters=[
                {"taskFile": LaunchConfiguration("taskFile")},
                {"referenceFile": LaunchConfiguration("referenceFile")},
                {"urdfFile": LaunchConfiguration("urdfFile")},
            ],
        ),
        Node(
            package="ocs2_hunter_ros",
            executable="hunter_target",
            name="legged_robot_target",
            output="screen",
            parameters=[
                {"referenceFile": LaunchConfiguration("referenceFile")},
            ],
        ),
        Node(
            package="ocs2_hunter_ros",
            executable="hunter_target_keepalive",
            name="legged_robot_target_keepalive",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enableKeepalive")),
            parameters=[
                {"referenceFile": LaunchConfiguration("referenceFile")},
                {"publishRate": 10.0},
            ],
        ),
        Node(
            package="ocs2_legged_robot_ros",
            executable="legged_robot_gait_command",
            name="legged_robot_gait_command",
            output="screen",
            parameters=[
                {"gaitCommandFile": LaunchConfiguration("gaitCommandFile")},
            ],
        ),
    ])
