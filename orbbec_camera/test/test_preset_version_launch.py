import importlib.util
from pathlib import Path
import tempfile
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILES = (
    "launch/gemini_301_series.launch.py",
    "launch/gemini_330_series.launch.py",
    "launch/gemini_330_series_low_cpu.launch.py",
    "launch/gemini_intra_process_demo_launch.py",
    "examples/multi_camera_shared_container/gemini_330_series_shared_container.launch.py",
    "examples/multi_camera_synced_verification_tool/gemini_330_series_synced_verify.launch.py",
    "examples/benchmark/gemini_330_series_benchmark.launch.py",
)


def load_launch(relative_path):
    path = PACKAGE_DIR / relative_path
    spec = importlib.util.spec_from_file_location("camera_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = [
        entity for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    context = LaunchContext()
    for arg in args:
        context.launch_configurations[arg.name] = perform_substitutions(context, arg.default_value)
    context.launch_configurations["config_file_path"] = ""
    return module, args, context


def evaluated_version(module, args, context):
    params = module.load_parameters(context, args)
    # Exercise launch_ros evaluation too: it can interpret numeric strings as YAML numbers.
    version = {"device_preset_version": params["device_preset_version"]}
    return evaluate_parameters(context, normalize_parameters([version]))[0]["device_preset_version"]


class PresetVersionLaunchTest(unittest.TestCase):
    def test_version_is_optional(self):
        for path in LAUNCH_FILES:
            with self.subTest(path=path):
                module, args, context = load_launch(path)
                self.assertIn("device_preset_version", {arg.name for arg in args})
                self.assertEqual(evaluated_version(module, args, context), "")

    def test_command_line_version_remains_a_string(self):
        for path in LAUNCH_FILES:
            for version in ("1", "1.0", "001", "1.2.3"):
                with self.subTest(path=path, version=version):
                    module, args, context = load_launch(path)
                    context.launch_configurations["device_preset_version"] = version
                    actual = evaluated_version(module, args, context)
                    self.assertEqual(actual, version)
                    self.assertIsInstance(actual, str)

    def test_yaml_version_preserves_its_string_value(self):
        for path in LAUNCH_FILES:
            for version in ("1.0", "001", "1.2.3"):
                with self.subTest(path=path, version=version):
                    module, args, context = load_launch(path)
                    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
                        yaml.safe_dump({"device_preset_version": version}, config)
                        config.flush()
                        context.launch_configurations["config_file_path"] = config.name
                        actual = evaluated_version(module, args, context)
                    self.assertEqual(actual, version)


if __name__ == "__main__":
    unittest.main()
