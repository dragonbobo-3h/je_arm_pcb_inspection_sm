"""Launch-description checks only: never execute a node or command a robot."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import mock_open, patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


ROOT = Path(__file__).resolve().parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "launch" / name)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestLoopModeLaunch(unittest.TestCase):
    def test_state_machine_defaults_to_normal_mode(self):
        module = load_launch("je_arm_state_machine.launch.py")
        with patch.object(module, "get_package_share_directory", return_value="/mock/share"), \
                patch.object(module, "load_yaml", return_value={}), \
                patch("builtins.open", mock_open(read_data="<robot/>")):
            description = module.generate_launch_description()
        argument = next(
            item for item in description.entities
            if isinstance(item, DeclareLaunchArgument) and item.name == "loop_mode"
        )
        context = LaunchContext()
        argument.execute(context)
        self.assertEqual(context.launch_configurations["loop_mode"], "false")

    def test_state_machine_passes_typed_boolean(self):
        module = load_launch("je_arm_state_machine.launch.py")
        for value, expected in (("false", False), ("true", True)):
            with self.subTest(value=value):
                # ParameterValue caches evaluation; each launch needs fresh objects.
                with patch.object(module, "get_package_share_directory", return_value="/mock/share"), \
                        patch.object(module, "load_yaml", return_value={}), \
                        patch("builtins.open", mock_open(read_data="<robot/>")), \
                        patch.object(module, "Node", wraps=module.Node) as node:
                    module.generate_launch_description()
                parameter = node.call_args.kwargs["parameters"][-1]["loop_mode"]
                context = LaunchContext()
                context.launch_configurations["loop_mode"] = value
                self.assertIs(parameter.evaluate(context), expected)

    def test_combined_launch_defaults_to_normal_mode(self):
        module = load_launch("je_arm_pcb_inspection_sm.launch.py")
        with patch.object(module, "get_package_share_directory", return_value="/mock/share"):
            description = module.generate_launch_description()
        argument = next(
            item for item in description.entities
            if isinstance(item, DeclareLaunchArgument) and item.name == "loop_mode"
        )
        context = LaunchContext()
        argument.execute(context)
        self.assertEqual(context.launch_configurations["loop_mode"], "false")

    def test_combined_launch_forwards_both_modes(self):
        module = load_launch("je_arm_pcb_inspection_sm.launch.py")
        with patch.object(module, "get_package_share_directory", return_value="/mock/share"):
            description = module.generate_launch_description()
        includes = [
            item for item in description.entities if isinstance(item, IncludeLaunchDescription)
        ]
        forwarded = next(
            dict(item.launch_arguments)["loop_mode"] for item in includes
            if "loop_mode" in dict(item.launch_arguments)
        )
        for value in ("false", "true"):
            with self.subTest(value=value):
                context = LaunchContext()
                context.launch_configurations["loop_mode"] = value
                self.assertEqual(forwarded.perform(context), value)


if __name__ == "__main__":
    unittest.main()