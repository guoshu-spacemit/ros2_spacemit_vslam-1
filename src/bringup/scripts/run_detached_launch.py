#!/usr/bin/env python3
"""Run a detached group of launch commands and supervise their lifetime."""

import argparse
import asyncio
import fcntl
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import stat
import subprocess
import sys
import time


CONTROL = Path("control.sock")
STATUS = Path("status.json")
STARTUP_WINDOW = 0.5
SIGNAL_TIMEOUTS = ((signal.SIGINT, 8), (signal.SIGTERM, 2), (signal.SIGKILL, 1))
STOP_TIMEOUT = sum(timeout for _, timeout in SIGNAL_TIMEOUTS) + 2
STARTUP_TIMEOUT = STOP_TIMEOUT + 2


def write_status(state, **details):
    temporary = STATUS.with_suffix(".new")
    with open_output(temporary) as output:
        output.write(json.dumps({"state": state, **details}, indent=2) + "\n")
    temporary.replace(STATUS)


def open_output(path, *, append=False):
    flags = os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK
    flags |= os.O_APPEND if append else os.O_TRUNC
    descriptor = os.open(path, flags, 0o600)
    if not stat.S_ISREG(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise OSError(f"Expected a regular file: {path}")
    return os.fdopen(descriptor, "a" if append else "w")


def process_groups_live(groups):
    """Include descendants after their leader exits, but ignore reaped/zombie tasks."""
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            fields = (entry / "stat").read_text(errors="replace").rsplit(")", 1)[1].split()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if fields[0] not in {"Z", "X"} and int(fields[2]) in groups:
            yield int(fields[2])


async def stop_children(children):
    groups = {process.pid for process in children.values()}
    for sig, timeout in SIGNAL_TIMEOUTS:
        groups.intersection_update(process_groups_live(groups))
        if not groups:
            break
        for group in groups:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while groups and time.monotonic() < deadline:
            await asyncio.sleep(0.05)
            groups.intersection_update(process_groups_live(groups))
    if groups:
        raise RuntimeError(f"Process groups still running after cleanup: {sorted(groups)}")
    for process in children.values():
        await process.wait()


async def supervise(commands, working_directory):
    stopping = asyncio.Event()
    stopped = asyncio.Event()
    responses = set()
    children = {}
    child_tasks = {}
    startup_reply_sent = False
    error = None
    result = {}
    roles = [name for name, _, _ in commands]

    async def request_control(reader, writer):
        task = asyncio.current_task()
        responses.add(task)
        try:
            request = await asyncio.wait_for(reader.readline(), timeout=1)
            if request == b"stop\n":
                stopping.set()
                await stopped.wait()
                writer.write(b"stopped\n" if result.get("cleanup_error") is None else b"failed\n")
                await writer.drain()
            elif request == b"status\n":
                writer.write(b"alive\n")
                await writer.drain()
        except (asyncio.TimeoutError, ConnectionResetError, BrokenPipeError):
            pass
        finally:
            writer.close()
            responses.discard(task)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopping.set)
    server = await asyncio.start_unix_server(request_control, path=CONTROL)
    stop_task = asyncio.create_task(stopping.wait())
    opened_logs = set()
    try:
        write_status("starting", roles=roles)
        for name, log, command in commands:
            if log not in opened_logs:
                open_output(log).close()
                opened_logs.add(log)
            with open_output(log, append=True) as output:
                try:
                    process = await asyncio.create_subprocess_exec(
                        *command, cwd=working_directory, start_new_session=True,
                        stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                    )
                except OSError as failure:
                    raise RuntimeError(f"Could not start {name}: {failure}. See {log}") from failure
            children[name] = process
            child_tasks[asyncio.create_task(process.wait())] = name
            with open_output(f"{name}.pid") as pid_file:
                pid_file.write(f"{process.pid}\n")

        watched = [stop_task, *child_tasks]
        finished, _ = await asyncio.wait(
            watched, timeout=STARTUP_WINDOW, return_when=asyncio.FIRST_COMPLETED,
        )
        if not finished:
            write_status("running", roles=roles, readiness="not_observed",
                         processes={name: process.pid for name, process in children.items()})
            print("submitted", flush=True)
            startup_reply_sent = True
            finished, _ = await asyncio.wait(watched, return_when=asyncio.FIRST_COMPLETED)
        if stop_task not in finished:
            failed = next(task for task in child_tasks if task in finished)
            name = child_tasks[failed]
            returncode = failed.result()
            log = next(log for role, log, _ in commands if role == name)
            result.update(process=name, returncode=returncode)
            error = f"{name} exited unexpectedly (code {returncode}). See {log}"
        elif not startup_reply_sent:
            error = "Startup was interrupted"
    except (OSError, RuntimeError) as failure:
        error = str(failure)
    finally:
        stop_task.cancel()
        try:
            await stop_children(children)
        except (OSError, RuntimeError) as failure:
            result["cleanup_error"] = str(failure)
            error = f"{error}: {failure}" if error else str(failure)
        else:
            for name in children:
                Path(f"{name}.pid").unlink(missing_ok=True)
        if error:
            result["reason"] = error
        try:
            write_status("failed" if error else "stopped", roles=roles, **result)
        finally:
            server.close()
            CONTROL.unlink(missing_ok=True)
            stopped.set()
            await asyncio.gather(*responses)
            await server.wait_closed()
            if not startup_reply_sent:
                print(error or "Startup was interrupted", flush=True)


def ask_control(request, timeout):
    try:
        control_info = CONTROL.lstat()
    except FileNotFoundError:
        return None
    if not stat.S_ISSOCK(control_info.st_mode):
        raise RuntimeError(f"Expected a control socket: {CONTROL}")
    with socket.socket(socket.AF_UNIX) as connection:
        connection.settimeout(timeout)
        try:
            connection.connect(str(CONTROL))
        except (ConnectionRefusedError, FileNotFoundError):
            return None
        connection.sendall(request)
        with connection.makefile("rb") as reply:
            return reply.readline()


def stop_existing():
    reply = ask_control(b"stop\n", STOP_TIMEOUT)
    if reply:
        if reply != b"stopped\n":
            raise RuntimeError(f"Could not stop the launcher. See {STATUS}")
        return
    CONTROL.unlink(missing_ok=True)

    # Legacy PID files did not record process identity. Never signal from them.
    roles = {"stack", "bridge", "live", "foxglove"}
    if STATUS.is_file():
        roles.update(json.loads(STATUS.read_text()).get("roles", []))
    pid_files = [Path(f"{name}.pid") for name in sorted(roles)
                 if re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_-]*", name)]
    groups = {}
    for pid_file in pid_files:
        if not pid_file.is_file() or pid_file.is_symlink():
            continue
        pid = pid_file.read_text().strip()
        if re.fullmatch(r"[1-9][0-9]{0,9}", pid) and int(pid) > 1:
            groups[int(pid)] = pid_file.name
    active = set(process_groups_live(groups)) if groups else set()
    if active:
        names = ", ".join(groups[group] for group in sorted(active))
        raise RuntimeError(f"Unmanaged process groups referenced by {names}; stop their original launcher")
    for pid_file in pid_files:
        pid_file.unlink(missing_ok=True)


def start(run_directory, working_directory, helper, arguments, before_start):
    stop_existing()
    if before_start:
        result = subprocess.run(before_start, cwd=working_directory, check=False)
        if result.returncode:
            reason = f"Pre-start command failed (code {result.returncode})"
            write_status("failed", reason=reason, returncode=result.returncode)
            raise RuntimeError(reason)
    with open_output("supervisor.log") as log:
        supervisor = subprocess.Popen(
            [sys.executable, helper, "supervise", run_directory, *arguments],
            cwd=working_directory, start_new_session=True, text=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=log,
        )
    with supervisor.stdout:
        try:
            ready, _, _ = select.select([supervisor.stdout], [], [], STARTUP_TIMEOUT)
            if not ready:
                raise RuntimeError("Startup timed out. See supervisor.log and status.json")
            reply = supervisor.stdout.readline().strip()
            if reply != "submitted":
                supervisor.wait(timeout=STOP_TIMEOUT)
                raise RuntimeError(reply or "Supervisor could not start. See supervisor.log")
        except (KeyboardInterrupt, RuntimeError):
            if supervisor.poll() is None:
                supervisor.terminate()
                supervisor.wait(timeout=STOP_TIMEOUT)
            raise


def parse_commands(arguments, parser):
    commands = []
    for item in arguments:
        if not commands or item == ":::":
            commands.append([])
        if item != ":::":
            commands[-1].append(item)
    names = set()
    for command in commands:
        if len(command) < 3:
            parser.error("Expected NAME LOGFILE COMMAND [ARG ...]")
        name, log = command[:2]
        if (not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_-]*", name) or name in names
                or not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_-]*\.log", log)
                or log == "supervisor.log"):
            parser.error("Expected distinct process names and simple .log filenames")
        names.add(name)
    if not names:
        parser.error("At least one command is required")
    return [(name, log, command) for name, log, *command in commands]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status", "prepare", "supervise"))
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    helper = Path(__file__).resolve()
    working_directory = Path.cwd()
    run_directory = args.run_directory.absolute()
    if args.action in {"stop", "status"} and not run_directory.exists() and not run_directory.is_symlink():
        if args.action == "status":
            print("No launch submitted.")
        return 0
    run_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    directory_info = run_directory.lstat()
    if (not stat.S_ISDIR(directory_info.st_mode) or directory_info.st_uid != os.geteuid()
            or directory_info.st_mode & 0o022):
        raise RuntimeError(f"Run directory must be owned by this user and protected from writes: {run_directory}")
    os.chdir(run_directory)
    if args.action == "prepare":
        return 0
    if args.action == "supervise":
        asyncio.run(supervise(parse_commands(args.command, parser), working_directory))
        return 0
    lock_flags = os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK
    with os.fdopen(os.open("control.lock", lock_flags, 0o600), "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("Another launcher command is in progress") from error
        if args.action == "stop":
            stop_existing()
        elif args.action == "status":
            if not STATUS.exists():
                print("No launch submitted.")
                return 0
            status = json.loads(STATUS.read_text())
            if (status["state"] in {"starting", "running"}
                    and ask_control(b"status\n", 2) != b"alive\n"):
                status.update(state="failed", reason="Launch supervisor is no longer running")
            print(json.dumps(status, indent=2))
            return int(status["state"] == "failed")
        else:
            arguments = args.command
            before_start = []
            if arguments and arguments[0] == "--before-start":
                if "--" not in arguments or arguments.index("--") < 2:
                    parser.error("Expected --before-start COMMAND [ARG ...] --")
                separator = arguments.index("--")
                before_start, arguments = arguments[1:separator], arguments[separator:]
            if arguments and arguments[0] == "--":
                arguments = arguments[1:]
            parse_commands(arguments, parser)
            start(run_directory, working_directory, helper, arguments, before_start)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        sys.exit(f"ERROR: {error}")
