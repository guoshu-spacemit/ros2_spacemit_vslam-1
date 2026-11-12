#!/usr/bin/env python3
"""Send a bounded one-shot command to the existing replay controller."""
import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("resume", "pause", "step"))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("replay_command")
    pub = node.create_publisher(String, "/replay/control", 10)
    state = {}

    def receive(message):
        state.clear()
        state.update(json.loads(message.data))

    node.create_subscription(String, "/replay/control_state", receive,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + 10
    try:
        while not pub.get_subscription_count() or not state:
            if time.monotonic() >= deadline:
                raise RuntimeError("No replay control subscriber; check session status/domain")
            rclpy.spin_once(node, timeout_sec=0.05)
        if state['state'] in ('finished', 'error'):
            raise RuntimeError(f"Replay is {state['state']}")
        if args.command == 'step' and state['state'] != 'paused':
            raise RuntimeError('Pause before single stepping')
        initial = state['acknowledged']
        pub.publish(String(data=args.command))
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            done = ((args.command == 'step' and state['acknowledged'] > initial)
                    or (args.command == 'pause' and state['state'] == 'paused')
                    or (args.command == 'resume' and state['state'] == 'playing'))
            if done or state['state'] == 'finished':
                print(json.dumps(state))
                break
            if state['state'] == 'error' or time.monotonic() >= deadline:
                raise RuntimeError(f'Command not completed: {state}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
