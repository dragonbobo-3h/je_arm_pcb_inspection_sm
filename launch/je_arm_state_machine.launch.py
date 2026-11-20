import importlib.util
import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def _load_mk2_support(moveit_config_share: Path):
    support_path = moveit_config_share / "launch" / "lh_mk2_support.py"
    spec = importlib.util.spec_from_file_location("lh_mk2_support", support_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load MK2 support module from {support_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_moveit_and_arm_params(context, moveit_config_share: Path):
    arm_mode = LaunchConfiguration("arm_mode").perform(context)
    model = LaunchConfiguration("model").perform(context)
    waist = LaunchConfiguration("waist").perform(context)
    use_fake_executor = LaunchConfiguration("use_fake_executor").perform(context).lower() == "true"
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    state_port = int(LaunchConfiguration("state_port").perform(context))
    command_port = int(LaunchConfiguration("command_port").perform(context))
    left_tip_override = LaunchConfiguration("left_tip_link").perform(context).strip()
    right_tip_override = LaunchConfiguration("right_tip_link").perform(context).strip()

    config_dir = moveit_config_share / "config"

    if arm_mode == "single":
        if waist == "feedback" and model != "dual":
            raise ValueError("waist:=feedback requires model:=dual")

        support = _load_mk2_support(moveit_config_share)
        if model == "dual":
            source_urdf, source_srdf = support.adapt_dual_model(
                (config_dir / "DUAL_JEARM.urdf").read_text(encoding="utf-8"),
                (config_dir / "DUAL_JEARM.srdf").read_text(encoding="utf-8"),
                waist=(waist == "feedback"),
            )
            flange_parent = "Left_Link7"
        else:
            source_urdf = (config_dir / "LH_MK2.urdf").read_text(encoding="utf-8")
            source_srdf = (config_dir / "LH_MK2.srdf").read_text(encoding="utf-8")
            flange_parent = "Link7"

        if not use_fake_executor:
            hardware = load_yaml(str(config_dir / "hardware_mk2.yaml"))
            robot_description, robot_description_semantic, joint_limits_yaml = (
                support.build_hardware_description(
                    source_urdf,
                    source_srdf,
                    hardware,
                    robot_ip,
                    state_port,
                    command_port,
                    control_backend="simulation",
                    confirm_physical_control=False,
                    shared_ownership=False,
                    flange_parent=flange_parent,
                )
            )
            default_left_tip = "sdk_flange"
        else:
            robot_description = source_urdf
            robot_description_semantic = source_srdf
            joint_limits_yaml = load_yaml(str(config_dir / "joint_limits_mk2.yaml"))
            default_left_tip = flange_parent

        kinematics_yaml = load_yaml(str(config_dir / "kinematics_single.yaml"))
        planning_yaml = load_yaml(str(config_dir / "ompl_planning_mk2.yaml"))
        arm_params = {
            "arm_mode": "single",
            "left_planning_group": "jearm",
            "right_planning_group": "",
            "both_planning_group": "jearm",
            "left_tip_link": left_tip_override or default_left_tip,
            "right_tip_link": "",
            "joint_states_dir": "config/move_group_client/joint_states/mk2_single",
        }
    else:
        robot_description = (config_dir / "DUAL_JEARM.urdf").read_text(encoding="utf-8")
        robot_description_semantic = (config_dir / "DUAL_JEARM.srdf").read_text(encoding="utf-8")
        kinematics_yaml = load_yaml(str(config_dir / "kinematics.yaml"))
        joint_limits_yaml = load_yaml(str(config_dir / "joint_limits.yaml"))
        planning_yaml = load_yaml(str(config_dir / "planning_pipelines.yaml"))

        default_left_tip = "Left_Link7" if "Left_Link7" in robot_description_semantic else "Link17"
        default_right_tip = "Right_Link7" if "Right_Link7" in robot_description_semantic else "Link27"
        arm_params = {
            "arm_mode": "dual",
            "left_planning_group": "left_arm",
            "right_planning_group": "right_arm",
            "both_planning_group": "both_arms",
            "left_tip_link": left_tip_override or default_left_tip,
            "right_tip_link": right_tip_override or default_right_tip,
            "joint_states_dir": "config/move_group_client/joint_states",
        }

    moveit_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics_yaml,
        "robot_description_planning": joint_limits_yaml,
    }
    if planning_yaml:
        moveit_params.update(planning_yaml)

    moveit_params["planning_plugin"] = "ompl_interface/OMPLPlanner"
    moveit_params["trajectory_execution"] = {
        "manage_controllers": True,
        "allowed_execution_timeout_scaling": 100.0,
        "execution_duration_monitoring": False,
        "allowed_goal_duration_margin": 5.0,
        "action_monitor_rate": 10.0,
    }
    moveit_params["start_state_max_bounds_error"] = 0.05
    moveit_params["allow_approximate_solution"] = True

    return moveit_params, arm_params


def _launch_setup(context):
    publish_static_obstacles = LaunchConfiguration("publish_static_obstacles")
    enable_virtual_grasp_boxes = LaunchConfiguration("enable_virtual_grasp_boxes")
    enable_gripper_control = LaunchConfiguration("enable_gripper_control")
    simulated_gripper_action_sec = LaunchConfiguration("simulated_gripper_action_sec")
    gripper_action_delay_sec = LaunchConfiguration("gripper_action_delay_sec")
    pcb_detection_topic = LaunchConfiguration("pcb_detection_topic")
    place_slot_topic = LaunchConfiguration("place_slot_topic")
    inspect_done_topic = LaunchConfiguration("inspect_done_topic")
    inspect_front_trigger_service = LaunchConfiguration("inspect_front_trigger_service")
    inspect_back_trigger_service = LaunchConfiguration("inspect_back_trigger_service")
    skip_inspect_handover = LaunchConfiguration("skip_inspect_handover")
    loop_mode = LaunchConfiguration("loop_mode")
    controlled_arm = LaunchConfiguration("controlled_arm")
    # Inherit ROS_DOMAIN_ID from the shell so the SM matches lh_mk2.launch.py.
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "0")

    runtime_env = {
        "PYTHONPATH": "",
        "PYTHONHOME": "",
        "CONDA_PREFIX": "",
        "CONDA_DEFAULT_ENV": "",
        "CONDA_PROMPT_MODIFIER": "",
        "ROS_DOMAIN_ID": ros_domain_id,
        "CONDA_SHLVL": "",
        "LD_PRELOAD": "",
        "TZ": "Asia/Shenzhen",
        "RCUTILS_CONSOLE_OUTPUT_FORMAT": "[{severity}] [{name}]: {message}",
        "PATH": ["/usr/bin:/bin:/usr/sbin:/sbin:", EnvironmentVariable("PATH", default_value="")],
        "LD_LIBRARY_PATH": [
            "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    }

    moveit_config_share = Path(get_package_share_directory("my_jearm_moveit_config"))
    moveit_params, arm_params = _build_moveit_and_arm_params(context, moveit_config_share)

    keyboard_server = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-lc",
            (
                "unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL && "
                "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH && "
                f"export ROS_DOMAIN_ID={ros_domain_id} && "
                "source /opt/ros/humble/setup.bash && "
                "source /home/test/ros2_ws/install/setup.bash && "
                "echo 'SMACC2 keyboard ready: s(start), n(next step), w(loop), p(pause), r(resume), b(back), f(fault), u(resource-unavailable)' && "
                "/usr/bin/python3 /home/test/ros2_ws/install/cl_keyboard/lib/cl_keyboard/keyboard_server_node.py; "
                "echo; echo 'keyboard_server_node exited'; "
                "read -r -n 1 -s -p 'Press any key to close...'"
            ),
        ],
        output="screen",
        shell=False,
        additional_env=runtime_env,
    )

    sm_params = {
        "publish_static_obstacles": ParameterValue(publish_static_obstacles, value_type=bool),
        "enable_virtual_grasp_boxes": ParameterValue(enable_virtual_grasp_boxes, value_type=bool),
        "enable_gripper_control": ParameterValue(enable_gripper_control, value_type=bool),
        "simulated_gripper_action_sec": ParameterValue(simulated_gripper_action_sec, value_type=float),
        "gripper_action_delay_sec": ParameterValue(gripper_action_delay_sec, value_type=float),
        "pcb_detection_topic": pcb_detection_topic,
        "place_slot_topic": place_slot_topic,
        "inspect_done_topic": inspect_done_topic,
        "inspect_front_trigger_service": inspect_front_trigger_service,
        "inspect_back_trigger_service": inspect_back_trigger_service,
        "skip_inspect_handover": ParameterValue(skip_inspect_handover, value_type=bool),
        "loop_mode": ParameterValue(loop_mode, value_type=bool),
        "controlled_arm": controlled_arm,
    }
    sm_params.update(arm_params)

    sm_node = TimerAction(
        period=5.0,  # This launch assumes move_group/control stack is already running elsewhere.
        actions=[
            Node(
                package="je_arm_pcb_inspection_sm",
                executable="je_arm_pcb_inspection_sm_node",
                output="screen",
                parameters=[moveit_params, sm_params],
                arguments=[
                    "--ros-args",
                    "--log-level", "warn",
                    "--log-level", "SmJeArmPcbInspection:=error",
                    "--log-level", "move_group_interface:=warn",
                    "--log-level", "je_arm_pcb_inspection_sm.biz:=info",
                ],
                additional_env=runtime_env,
            )
        ],
    )

    return [
        LogInfo(
            msg=(
                f"JE_ARM SM launch | arm_mode={arm_params['arm_mode']} "
                f"model={LaunchConfiguration('model').perform(context)} "
                f"waist={LaunchConfiguration('waist').perform(context)} "
                f"left_group={arm_params['left_planning_group']} "
                f"left_tip={arm_params['left_tip_link']} "
                f"joint_states_dir={arm_params['joint_states_dir']} "
                f"ROS_DOMAIN_ID={ros_domain_id}"
            )
        ),
        SetEnvironmentVariable(
            name="JE_ARM_CONTROLLED_ARM",
            value=controlled_arm,
        ),
        keyboard_server,
        sm_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_mode",
                default_value="single",
                choices=["single", "dual"],
                description="single: MK2 single-arm (jearm, joint1..7); dual: dual-arm (left_arm/right_arm/both_arms)",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="dual",
                choices=["dual", "lh_mk2"],
                description="For arm_mode:=single, matches lh_mk2.launch.py model argument (dual: DUAL_JEARM waist URDF; lh_mk2: single-arm URDF)",
            ),
            DeclareLaunchArgument(
                "waist",
                default_value="fixed",
                choices=["fixed", "feedback"],
                description="For arm_mode:=single with model:=dual, matches lh_mk2.launch.py waist argument",
            ),
            DeclareLaunchArgument(
                "use_fake_executor",
                default_value="false",
                choices=["true", "false"],
                description="false: hardware model with sdk_flange (matches lh_mk2.launch.py default); true: offline CAD model",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.99"),
            DeclareLaunchArgument("state_port", default_value="8000"),
            DeclareLaunchArgument("command_port", default_value="8001"),
            DeclareLaunchArgument(
                "left_tip_link",
                default_value="",
                description="Optional override for left arm end-effector tip link (empty = auto)",
            ),
            DeclareLaunchArgument(
                "right_tip_link",
                default_value="",
                description="Optional override for right arm end-effector tip link in dual mode (empty = auto)",
            ),
            DeclareLaunchArgument(
                "publish_static_obstacles",
                default_value="false",
                description="Whether the SM should publish config/obstacles.yaml into the planning scene",
            ),
            DeclareLaunchArgument(
                "enable_virtual_grasp_boxes",
                default_value="false",
                description="Whether the SM should create cl_moveit2z virtual grasp boxes on arm startup",
            ),
            DeclareLaunchArgument(
                "enable_gripper_control",
                default_value="true",
                description="Whether to send real gripper commands; false simulates each open/close action",
            ),
            DeclareLaunchArgument(
                "simulated_gripper_action_sec",
                default_value="2.0",
                description="Simulated duration for a gripper open/close action when gripper control is disabled",
            ),
            DeclareLaunchArgument(
                "gripper_action_delay_sec",
                default_value="0.3",
                description="Optional extra settle time after a gripper action reaches its completion condition; applies in real mode and overrides simulated duration when > 0",
            ),
            DeclareLaunchArgument(
                "pcb_detection_topic",
                default_value="/pcb_detection",
                description="Topic for incoming PCB detection messages",
            ),
            DeclareLaunchArgument(
                "place_slot_topic",
                default_value="/recommended_slot",
                description="Topic for incoming place-slot detection messages",
            ),
            DeclareLaunchArgument(
                "inspect_done_topic",
                default_value="/vision/inspect_done",
                description="Topic for inspect-done acknowledgements",
            ),
            DeclareLaunchArgument(
                "inspect_front_trigger_service",
                default_value="/vision/inspect_front",
                description="ROS Trigger service called when front-side inspection is ready; if unavailable, manual 'n' and inspect_done topic remain usable",
            ),
            DeclareLaunchArgument(
                "inspect_back_trigger_service",
                default_value="/vision/inspect_back",
                description="ROS Trigger service called when back-side inspection is ready; if unavailable, manual 'n' and inspect_done topic remain usable",
            ),
            DeclareLaunchArgument(
                "skip_inspect_handover",
                default_value="true",
                description="After front-side inspection, skip handovers and continue directly to placement (always true in arm_mode:=single)",
            ),
            DeclareLaunchArgument(
                "loop_mode",
                default_value="true",
                description="Keep the PCB gripped at PLACE, return it to the saved PICK pose, then repeat after returning to the start pose",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "controlled_arm",
                default_value="left",
                description="Arm command output: both, left, or right. The other arm's joint and gripper commands are blocked.",
                choices=["both", "left", "right"],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
