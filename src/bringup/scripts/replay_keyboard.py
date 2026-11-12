#!/usr/bin/env python3
"""Attach a terminal keyboard to an existing forward-only session replay."""

import argparse
from contextlib import ExitStack
import curses
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty


STATES = {"initializing", "playing", "pause_requested", "paused", "stepping",
          "fast_forward", "finishing", "finished", "error"}


class KeyParser:
    """Parse split ANSI/SS3 arrow sequences without treating Left as control."""

    def __init__(self):
        self.escape = bytearray()

    def feed(self, data):
        commands = []
        for value in data:
            if self.escape:
                if len(self.escape) == 1:
                    if value in (ord("["), ord("O")):
                        self.escape.append(value)
                        continue
                    self.escape.clear()
                else:
                    self.escape.append(value)
                    if 0x40 <= value <= 0x7e:
                        if value == ord("B"):
                            commands.append("step")
                        elif value == ord("C"):
                            commands.append("fast_forward")
                        self.escape.clear()
                    elif len(self.escape) > 32:
                        self.escape.clear()
                    continue
            if value == 0x1b:
                self.escape.append(value)
            elif value == ord(" "):
                commands.append("toggle")
            elif value in (ord("q"), ord("Q"), 0x03):
                commands.append("quit")
        return commands


class TerminalInput:
    """Keep signal keys enabled and restore terminal settings on every exit."""

    def __init__(self, fd):
        self.fd = fd
        self.saved = None
        self.parser = KeyParser()

    def __enter__(self):
        if not os.isatty(self.fd):
            raise RuntimeError("Keyboard control requires a TTY; connect with ssh -t.")
        self.saved = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd, termios.TCSANOW)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            raise
        return self

    def __exit__(self, *_):
        if self.saved is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            except termios.error:
                # A disconnected SSH terminal may already have disappeared.
                pass

    def read(self, timeout=0.05):
        if not select.select([self.fd], [], [], timeout)[0]:
            return []
        try:
            data = os.read(self.fd, 128)
        except OSError:
            return None
        return self.parser.feed(data) if data else None


def parse_state(data):
    state = json.loads(data)
    if not isinstance(state, dict) or state.get("state") not in STATES:
        raise ValueError("unknown replay state")
    for field in ("acknowledged", "total_targets"):
        if type(state.get(field)) is not int or state[field] < 0:
            raise ValueError(f"invalid {field}")
    position = state.get("position_sec")
    if (isinstance(position, bool) or not isinstance(position, (int, float))
            or not math.isfinite(position) or position < 0):
        raise ValueError("invalid source-time position")
    amount = state.get("fast_forward_sec", 5.0)
    if (isinstance(amount, bool) or not isinstance(amount, (int, float))
            or not math.isfinite(amount) or amount <= 0):
        raise ValueError("invalid fast-forward interval")
    return state


def state_line(state):
    frame = state.get("frame_index")
    if frame is None:
        frame = "-"
    return (f"{state['state']} | source {state['position_sec']:.3f}s | "
            f"VIO {state['acknowledged']}/{state['total_targets']} | frame {frame}")


def positive_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-topic", default="/replay/control")
    parser.add_argument("--state-topic", default="/replay/control_state")
    parser.add_argument("--connect-timeout-sec", type=positive_seconds, default=30.0)
    parser.add_argument("--result-file", type=Path,
                        help="Write the last observed state and detach reason for the controller")
    parser.add_argument("--ui", choices=("plain", "tui"), default="plain")
    options, ros_args = parser.parse_known_args()
    if not sys.stdin.isatty():
        print("Keyboard control requires a TTY; connect with ssh -t.", file=sys.stderr)
        return 2

    # Import ROS only when attaching; terminal parsing can be verified on hosts.
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f"replay_keyboard_{os.getpid()}")
    command_qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, options.control_topic, command_qos)
    received = {"state": None, "invalid": False}

    def receive(message):
        try:
            received["state"] = parse_state(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            received["invalid"] = True

    node.create_subscription(String, options.state_topic, receive, state_qos)
    old_handlers = {}
    exit_reason = "ros_shutdown"

    def detach(signum, *_):
        nonlocal exit_reason
        exit_reason = {signal.SIGINT: "interrupt", signal.SIGTERM: "terminated",
                       signal.SIGHUP: "terminal_closed"}[signum]
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        old_handlers[signum] = signal.signal(signum, detach)
    started = time.monotonic()
    connected = False
    missing_since = None
    rendered_at = 0.0
    previous_state = None
    result = 0
    dashboard = None

    def notice(english, chinese):
        if dashboard is not None:
            dashboard.notice = chinese
        else:
            print(english, flush=True)

    if options.ui == "plain":
        print("Space: pause/resume | Down: next VIO target while paused | "
              "Right: fast forward | q/Ctrl-C: detach (replay keeps running)", flush=True)
        print("Waiting for the replay node...", flush=True)
    try:
        with ExitStack() as stack:
            if options.ui == "tui":
                from terminal_ui import ReplayDashboard
                dashboard = stack.enter_context(ReplayDashboard())
                terminal = dashboard
            else:
                terminal = stack.enter_context(TerminalInput(sys.stdin.fileno()))
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                now = time.monotonic()
                state = received["state"]
                available = (node.count_publishers(options.state_topic) > 0
                             and publisher.get_subscription_count() > 0)
                if available and state is not None:
                    connected = True
                    missing_since = None
                elif connected:
                    if missing_since is None:
                        missing_since = now
                    if now - missing_since >= 2.0:
                        notice("\nReplay node disconnected; keyboard detached. "
                               "Check the replay log and ROS_DOMAIN_ID.", "连接已断开，请查看任务状态。")
                        result = 1
                        exit_reason = "disconnected"
                        break
                elif now - started >= options.connect_timeout_sec:
                    notice("\nNo replay control endpoint found; keyboard detached. "
                           "Check startup, ROS_DOMAIN_ID and the deployed replay version.",
                           "连接超时，请查看任务状态和 ROS 域。")
                    result = 1
                    exit_reason = "connect_timeout"
                    break
                if received["invalid"]:
                    notice("\nInvalid replay state received; check the deployed replay version.",
                           "回放状态格式无效，请检查运行版本。")
                    result = 1
                    exit_reason = "invalid_state"
                    break
                state_name = state["state"] if state is not None else None
                if now - rendered_at >= 0.25 or state_name != previous_state:
                    if dashboard is not None:
                        if state_name != previous_state:
                            dashboard.notice = {
                                "paused": "按空格继续播放，或 ↓ 前进一个原 VIO 输入帧。",
                                "playing": "按空格暂停；Q 打开操作页，不停止回放。",
                                "stepping": "正在等待本帧 VIO 完成；不跳过输入帧。",
                            }.get(state_name, "进度以 VIO 完成确认为准，建图与显示独立运行。")
                        dashboard.draw(state, available)
                    elif state is not None:
                        print("\r\033[K" + state_line(state), end="", flush=True)
                    rendered_at = now
                    previous_state = state_name
                if state is not None:
                    if state["state"] in ("finished", "error"):
                        notice("\nReplay " + state["state"] + ".", "回放已结束，正在返回操作页。")
                        result = int(state["state"] == "error")
                        exit_reason = state["state"]
                        break
                keys = terminal.read()
                if keys is None:
                    notice("\nTerminal closed; keyboard detached.", "终端连接已关闭。")
                    exit_reason = "terminal_closed"
                    break
                quit_requested = False
                for command in keys:
                    if command == "quit":
                        quit_requested = True
                        break
                    if not available or state is None:
                        notice("\nReplay is not connected yet; command ignored.",
                               "尚未连接；此次按键未发送，请等待节点就绪。")
                        continue
                    if command == "step" and state["state"] != "paused":
                        notice("\nDown requires a paused replay.", "请先按空格暂停，再按 ↓ 单步。")
                        continue
                    message = String()
                    message.data = command
                    publisher.publish(message)
                    amount = state.get("fast_forward_sec", 5.0)
                    label = command
                    if command == "fast_forward":
                        label = f"fast_forward (+{amount:g}s source time)"
                    chinese = {"toggle": "已请求播放/暂停，等待回放节点确认。",
                               "step": "已请求单帧，等待处理完成。",
                               "fast_forward": f"已请求前进 {amount:g} 秒，仍处理沿途所有目标帧。"}
                    notice(f"\nCommand: {label}", chinese[command])
                if quit_requested:
                    exit_reason = "user_detach"
                    break
    except KeyboardInterrupt:
        if exit_reason == "ros_shutdown":
            exit_reason = "interrupt"
    except (RuntimeError, curses.error) as error:
        print(str(error), file=sys.stderr)
        result = 2
        exit_reason = "client_error"
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_reason in ("user_detach", "interrupt") and options.ui == "plain":
        print("\n已退出键盘控制，未停止后台任务；暂停/播放状态保持不变。", flush=True)
        if not options.result_file:
            print("通过 ros2 run bringup run_vio_session.sh control 重新接管；空格播放/暂停，↓逐帧。", flush=True)
    if options.result_file:
        try:
            # One unique report per attachment; never overwrite an older report.
            with options.result_file.open("x") as output:
                json.dump({"reason": exit_reason, "exit_code": result,
                           "last_state": received["state"]}, output)
                output.write("\n")
        except OSError as error:
            print(f"Unable to save keyboard result: {error}", file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
