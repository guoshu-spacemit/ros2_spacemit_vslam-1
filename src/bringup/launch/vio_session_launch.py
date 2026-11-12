"""Reuse the reconstruction graph and add optional original-stream recording."""

import json
import os
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


def validate_options(options, launch_file):
    """Validate active node parameters and return the evaluated VIO parameters.

    Both launch entry points use the same argument declarations, conditions and
    parameter definitions. Preflight evaluates them without starting processes.
    The result is None when VIO is disabled.
    """
    module = runpy.run_path(str(launch_file))
    declarations = module["reconstruction_arguments"]()
    unknown = set(options) - {argument.name for argument in declarations}
    if unknown:
        raise ValueError(f"Unknown reconstruction arguments: {sorted(unknown)}")
    context = LaunchContext()
    context.launch_configurations.update(options)
    for argument in declarations:
        if argument.condition is None or argument.condition.evaluate(context):
            argument.execute(context)
    # The real graph derives node conditions/prefixes through this pure config
    # function before any Node executes. Reuse it rather than inventing defaults
    # or evaluating derived launch state before it exists.
    for action in module["configure_execution"](context):
        action.execute(context)
    vio_parameters = None
    for node_options in module["reconstruction_node_options"]():
        condition = node_options.get("condition")
        if condition is not None and not condition.evaluate(context):
            continue
        parameters = normalize_parameters(node_options["parameters"])
        values = evaluate_parameters(context, parameters)[0]
        if node_options["executable"] == "vio_node":
            vio_parameters = values
            if type(values["imu_frequency_hz"]) is not float:
                raise TypeError("imu_frequency_hz must be decimal (e.g. 0.0)")
        if "render_fps" in values and type(values["render_fps"]) is not float:
            raise TypeError("render_fps must be decimal (e.g. 60.0)")
    return vio_parameters


def generate_launch_description():
    """Snapshot the active VIO parameters before launching the session."""
    directory = Path(os.environ["SPACEMIT_ROBOT_VIO_SESSION_DIR"])
    session = json.loads((directory / "session.json").read_text())
    options = json.loads((directory / "launch-options.json").read_text())
    main_launch = (Path(get_package_share_directory("bringup")) /
                   "launch/vio_las2_esdf_launch.py")
    parameters = validate_options(options, main_launch)
    (directory / "vio_parameters.yaml").write_text(yaml.safe_dump(parameters))
    actions = []
    if session.get("agc", False):
        actions.append(Node(
            package="camera_io", executable="camera_agc_node", name="camera_agc",
            output="screen", parameters=[{
                **session["agc_parameters"], "log_file": str(directory / "agc.csv"),
            }],
        ))
    if session["record"]:
        actions.append(Node(
            package="camera_io", executable="recorder_node", name="recorder",
            output="screen", parameters=[{
                **session["recorder_parameters"],
                "config_file": str(directory / "recorder.json"),
                "session_directory": str(directory),
            }],
        ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch)), launch_arguments=options.items(),
    ))
    return LaunchDescription(actions)
