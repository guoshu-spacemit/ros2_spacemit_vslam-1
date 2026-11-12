#!/usr/bin/env python3
"""Supervise a long-running ROS launch; propagate unexpected child exits."""

import argparse
import json
import os
from pathlib import Path
import sys
import time

from launch import LaunchDescription, LaunchService
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


class PipelineStatus:
    """Session-local status, retained even if systemd collects the unit."""

    def __init__(self, path):
        self.path = Path(path)
        self.failed = False
        self.started = set()

    def write(self, state, **details):
        record = {"state": state, "time_unix": time.time(), **details}
        temporary = self.path.with_name(self.path.name + ".new")
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        os.replace(temporary, self.path)

    def on_exit(self, event, context):
        if context.is_shutdown or self.failed:
            return []
        self.failed = True
        reason = f"Unexpected exit: {event.process_name}, code={event.returncode}"
        self.write(
            "failed", reason=reason, process=event.process_name,
            pid=event.pid, returncode=event.returncode,
        )
        print(reason, file=sys.stderr, flush=True)
        return [EmitEvent(event=Shutdown(reason=reason))]

    def on_start(self, event, context):
        self.started.add(event.action)
        return []

    def on_complete(self, event, context):
        # Humble logs spawn failures but emits no ProcessExited for them.
        if context.is_shutdown or self.failed or event.action in self.started:
            return []
        self.failed = True
        details = event.action.process_details or {}
        name = details.get("name", "unknown child")
        reason = f"Failed to start: {name}; see launch log"
        self.write("failed", reason=reason, process=name)
        print(reason, file=sys.stderr, flush=True)
        return [EmitEvent(event=Shutdown(reason=reason))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status-file", required=True)
    parser.add_argument("launch_file", type=Path)
    parser.add_argument("launch_arguments", nargs="*", help="name:=value")
    args = parser.parse_args()
    options = {}
    for item in args.launch_arguments:
        key, separator, value = item.partition(":=")
        if not key or not separator:
            parser.error(f"Expected name:=value: {item}")
        options[key] = value
    if not args.launch_file.is_file():
        parser.error(f"Launch file missing: {args.launch_file}")

    status = PipelineStatus(args.status_file)
    status.path.parent.mkdir(parents=True, exist_ok=True)
    status.write("running", supervisor_pid=os.getpid(), readiness="not_observed")
    service = LaunchService()
    # Register before visiting the nested launch, so that another on-exit
    # handler cannot hide a failure by requesting shutdown first.
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=status.on_exit)),
        RegisterEventHandler(OnProcessStart(on_start=status.on_start)),
        RegisterEventHandler(OnExecutionComplete(
            target_action=lambda action: isinstance(action, ExecuteProcess),
            on_completion=status.on_complete)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(args.launch_file)),
                                 launch_arguments=options.items()),
    ]))
    result = 1
    try:
        result = service.run()
    except Exception as error:
        status.failed = True
        status.write("failed", reason=str(error))
        raise
    finally:
        if not status.failed:
            status.write(
                "stopped" if result == 0 else "failed",
                launch_returncode=result,
            )
    return 1 if status.failed else result


if __name__ == "__main__":
    sys.exit(main())
