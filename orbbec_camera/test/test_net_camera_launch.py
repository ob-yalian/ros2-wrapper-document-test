import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def load_module(path):
    spec = importlib.util.spec_from_file_location("camera_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def walk_entities(entities):
    for entity in entities:
        yield entity
        if isinstance(entity, IncludeLaunchDescription):
            continue
        children = entity.actions if isinstance(entity, TimerAction) else entity.get_sub_entities()
        yield from walk_entities(children)


class NetCameraLaunchTest(unittest.TestCase):
    def test_network_examples_select_direct_connection(self):
        for relative_path in (
            "examples/multi_net_camera/multi_net_camera.launch.py",
            "examples/action_command/multi_action_command.launch.py",
        ):
            with self.subTest(path=relative_path):
                module = load_module(PACKAGE_DIR / relative_path)
                with patch.object(
                    module, "get_package_share_directory", return_value=str(PACKAGE_DIR)
                ):
                    description = module.generate_launch_description()
                includes = [
                    entity for entity in walk_entities(description.entities)
                    if isinstance(entity, IncludeLaunchDescription)
                ]
                self.assertEqual(len(includes), 2)
                for include in includes:
                    values = dict(include.launch_arguments)
                    context = LaunchContext()

                    def evaluate(value):
                        return perform_substitutions(
                            context, normalize_to_list_of_substitutions(value))
                    self.assertEqual(evaluate(values["enumerate_net_device"]), "false")
                    self.assertTrue(evaluate(values["net_device_ip"]))
                    self.assertEqual(evaluate(values["net_device_port"]), "8090")

    def test_endpoint_parameters_keep_expected_types(self):
        for name in (
            "gemini_330_series", "gemini_330_series_low_cpu", "gemini_330_series_sdk_json"
        ):
            with self.subTest(launch=name):
                module = load_module(PACKAGE_DIR / "launch" / (name + ".launch.py"))
                args = [
                    entity for entity in module.generate_launch_description().entities
                    if isinstance(entity, DeclareLaunchArgument)
                ]
                context = LaunchContext()
                for arg in args:
                    context.launch_configurations[arg.name] = perform_substitutions(
                        context, arg.default_value)
                context.launch_configurations.update(
                    enumerate_net_device="false", net_device_ip="192.168.1.10", net_device_port="8090"
                )
                params = module.load_parameters(context, args)
                self.assertIs(params["enumerate_net_device"], False)
                self.assertEqual(params["net_device_ip"], "192.168.1.10")
                self.assertEqual(params["net_device_port"], 8090)
                self.assertIsInstance(params["net_device_port"], int)


if __name__ == "__main__":
    unittest.main()
