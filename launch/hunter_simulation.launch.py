import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare launch arguments
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="joint_trajectory_controller",
        description="Controller to spawn (joint_trajectory_controller or forward_command_controller)"
    )
    
    # Get package paths
    description_pkg_path = get_package_share_directory("legged_hunter_description")
    controllers_pkg_path = get_package_share_directory("legged_hunter_controllers")
    
    # Path to the XACRO file with ros2_control tags
    xacro_file = os.path.join(description_pkg_path, "urdf", "hunter_ros2_control.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # Path to the MJCF scene file
    mjcf_file = os.path.join(description_pkg_path, "mujoco", "hunter_scene.xml")

    # Controller config from legged_hunter_controllers package
    controller_config = os.path.join(controllers_pkg_path, "config", "hunter_controllers.yaml")

    # Mujoco Control Node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            {"robot_model_path": mjcf_file},
            controller_config
        ],
        output="screen"
    )

    # Spawn Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # Spawn Controller proportional to the argument
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("controller")],
        output="screen"
    )

    return LaunchDescription([
        controller_arg,
        mujoco_node,
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[controller_spawner],
            )
        )
    ])
