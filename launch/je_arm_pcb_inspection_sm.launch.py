from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_control_stack = LaunchConfiguration("start_control_stack")
    start_state_machine = LaunchConfiguration("start_state_machine")
    use_fake_executor = LaunchConfiguration("use_fake_executor")
    skip_inspect_handover = LaunchConfiguration("skip_inspect_handover")

    this_pkg_share = get_package_share_directory("je_arm_pcb_inspection_sm")
    moveit_pkg_share = get_package_share_directory("my_jearm_moveit_config")

    control_stack_launch = os.path.join(
        moveit_pkg_share,
        "launch",
        "je_arm_control_stack.launch.py",
    )

    state_machine_launch = os.path.join(
        this_pkg_share,
        "launch",
        "je_arm_state_machine.launch.py",
    )

    control_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_stack_launch),
        condition=IfCondition(start_control_stack),
        launch_arguments={"use_fake_executor": use_fake_executor}.items(),
    )

    state_machine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_machine_launch),
        condition=IfCondition(start_state_machine),
        launch_arguments={"skip_inspect_handover": skip_inspect_handover}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_control_stack",
                default_value="true",
                description="Start MoveIt control stack (real hardware or fake mode, determined by use_fake_executor)",
            ),
            DeclareLaunchArgument(
                "start_state_machine",
                default_value="true",
                description="Start SMACC2 state machine + keyboard",
            ),
            DeclareLaunchArgument(
                "use_fake_executor",
                default_value="true",
                description="Start fake trajectory executor (set true for simulation without ros2_control)",
            ),
            DeclareLaunchArgument(
                "skip_inspect_handover",
                default_value="false",
                description="After front-side inspection, skip handovers and continue directly to placement",
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=["/usr/lib/x86_64-linux-gnu:", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
            ),
            control_stack,
            state_machine,
        ]
    )

