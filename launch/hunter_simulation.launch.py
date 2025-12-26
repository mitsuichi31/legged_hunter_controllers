import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package paths
    description_pkg_path = get_package_share_directory('legged_hunter_description')
    controllers_pkg_path = get_package_share_directory('legged_hunter_controllers')
    
    # Path to the XACRO file with ros2_control tags
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'hunter_ros2_control.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Path to the MJCF scene file
    # Note: mujoco_ros2_control needs the absolute path to the xml
    # Using the one in the source directory for now to be safe, or one installed
    # Identifying the path via finding package is safer but the mujoco files were not installed by CMakeLists yet
    # We should update CMakeLists to install mujoco/model folder or refer to source
    # For now, let's point to the source location as we are in a container dev env
    mjcf_file = "/root/workspace/hunter_bipedal_control/mujoco/model/hunter/hunter_scene.xml"

    # Controller config from legged_hunter_controllers package
    controller_config = os.path.join(controllers_pkg_path, 'config', 'hunter_controllers.yaml')

    # Mujoco Control Node
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        parameters=[
            robot_description,
            {'robot_model_path': mjcf_file},
            controller_config
        ],
        output='screen'
    )

    # Spawn Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn Joint Trajectory Controller (PD Control)
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        mujoco_node,
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        )
    ])
