import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    controllers_pkg = get_package_share_directory("legged_hunter_controllers")
    wbc_pkg = get_package_share_directory("legged_wbc")

    delay_arg = DeclareLaunchArgument(
        "wbc_delay",
        default_value="0.0",
        description="Seconds to wait before starting WBC + simulation.",
    )
    wbc_start_delay_arg = DeclareLaunchArgument(
        "wbc_start_delay",
        default_value="0.0",
        description="Seconds to wait before starting WBC node.",
    )
    debug_fall_log_arg = DeclareLaunchArgument(
        "debug_fall_log",
        default_value="false",
        description="Enable detailed instability logging in WBC.",
    )
    fall_log_rp_arg = DeclareLaunchArgument(
        "fall_log_roll_pitch_threshold",
        default_value="0.5",
        description="Roll/pitch threshold (rad) to trigger instability logs.",
    )
    fall_log_z_arg = DeclareLaunchArgument(
        "fall_log_z_threshold",
        default_value="0.4",
        description="Z threshold (m) to trigger instability logs.",
    )
    com_target_x_arg = DeclareLaunchArgument(
        "com_target_x",
        default_value="0.10",
        description="COM target x offset (meters, positive is forward).",
    )
    weight_com_arg = DeclareLaunchArgument(
        "weight_com",
        default_value="80.0",
        description="COM task weight.",
    )
    reference_file_arg = DeclareLaunchArgument(
        "reference_file",
        default_value=os.path.join(controllers_pkg, "config", "ocs2", "reference.info"),
        description="Reference info file for defaultJointState.",
    )
    wait_controller_arg = DeclareLaunchArgument(
        "wait_for_controller",
        default_value="false",
        description="Wait for forward_command_controller before enabling WBC.",
    )

    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controllers_pkg, "launch", "hunter_ocs2_sqp.launch.py")
        )
    )

    wbc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wbc_pkg, "launch", "wbc_standing.launch.py")
        ),
        launch_arguments={
            "debug_fall_log": LaunchConfiguration("debug_fall_log"),
            "fall_log_roll_pitch_threshold": LaunchConfiguration("fall_log_roll_pitch_threshold"),
            "fall_log_z_threshold": LaunchConfiguration("fall_log_z_threshold"),
            "com_target_x": LaunchConfiguration("com_target_x"),
            "weight_com": LaunchConfiguration("weight_com"),
            "reference_file": LaunchConfiguration("reference_file"),
            "wbc_start_delay": LaunchConfiguration("wbc_start_delay"),
            "wait_for_controller": LaunchConfiguration("wait_for_controller"),
        }.items(),
    )

    delayed_wbc = TimerAction(
        period=LaunchConfiguration("wbc_delay"),
        actions=[wbc_launch],
    )

    return LaunchDescription([
        delay_arg,
        debug_fall_log_arg,
        fall_log_rp_arg,
        fall_log_z_arg,
        com_target_x_arg,
        weight_com_arg,
        reference_file_arg,
        wbc_start_delay_arg,
        wait_controller_arg,
        mpc_launch,
        delayed_wbc,
    ])
