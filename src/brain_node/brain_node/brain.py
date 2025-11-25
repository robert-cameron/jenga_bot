#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import Float32, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from manipulation.action import Manipulation


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # === Parameters ===
        self.declare_parameter('threshold_g', 80.0)
        self.threshold_g = float(self.get_parameter('threshold_g').value)

        # Cooldown duration after trigger (seconds)
        self.cooldown_s = 3

        # Timestamp of last trigger
        self.last_trigger_time = None

        # === I/O ===
        self.force_sub = self.create_subscription(
            Float32,
            '/prongs/force_g',
            self.on_force,
            10
        )

        self.stop_pub = self.create_publisher(
            Bool,
            '/safety/stop',
            10
        )

        # startup action client
        self.manipulation_client = ActionClient(
            self,
            Manipulation,
            '/manipulation_action'
        )
        # start the action chain once the executor is spinning.
        self.sequence_timer = self.create_timer(0.5, self._start_sequence)
        self.sequence_thread = None

        self.get_logger().info(
            f'force_stopper up. Watching /prongs/force_g > {self.threshold_g:.1f} g.'
        )

    def _start_sequence(self):
        """Kick off the move sequence once after startup."""
        self.sequence_timer.cancel()
        self.sequence_thread = threading.Thread(
            target=self._run_sequence,
            daemon=True,
        )
        self.sequence_thread.start()

    def _run_sequence(self):

        self.get_logger().info('Waiting for /manipulation_action server...')
        if not self.manipulation_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Manipulation action server not available; aborting sequence.')
            return

        while rclpy.ok():  # TODO: Add blocker to wait for robot's turn
            push_tf, pull_tf, place_tf = self.get_next_blocks()

            moves = [
                ('push_move', push_tf),
                ('pull_move', pull_tf),
                ('place_move', place_tf),
            ]

            for action_type, tf in moves:
                if not tf:
                    self.get_logger().error(f'TF frame missing for {action_type}; aborting sequence.')
                    return

                self.get_logger().info(f'Sending {action_type} targeting {tf}...')
                success = self._send_goal_and_wait(action_type, tf)
                if not success:
                    self.get_logger().error(f'{action_type} failed; stopping sequence.')
                    return

            self.get_logger().info('Push, pull, place cycle complete. Starting next cycle...')

    def _send_goal_and_wait(self, action_type: str, tf: str) -> bool:
        """Send a Manipulation goal and wait for the result."""

        goal_msg = Manipulation.Goal()
        goal_msg.action_type = action_type
        goal_msg.tf = tf

        goal_sent = threading.Event()
        goal_result = threading.Event()
        outcome = {'accepted': False, 'succeeded': False}

        def _on_result(future):
            try:
                result = future.result().result
                outcome['succeeded'] = bool(result.result)
            except Exception as exc:
                self.get_logger().error(f'Error waiting for {action_type} result: {exc}')
            finally:
                goal_result.set()

        def _on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn(f'{action_type} was rejected by the server.')
                    return

                outcome['accepted'] = True
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(_on_result)
            except Exception as exc:
                self.get_logger().error(f'Error sending {action_type}: {exc}')
            finally:
                goal_sent.set()

        send_goal_future = self.manipulation_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(_on_goal_response)

        # wait for acceptance
        goal_sent.wait()
        if not outcome['accepted']:
            return False

        # wait for result
        goal_result.wait()
        return outcome['succeeded']


    def on_force(self, msg: Float32):
        now = self.get_clock().now()
        force = msg.data

        # === Ignore readings during cooldown ===
        if self.last_trigger_time is not None:
            if (now - self.last_trigger_time) < Duration(seconds=self.cooldown_s):
                # Still in the cooldown window → ignore force
                return
            else:
                # Cooldown done → clear
                self.last_trigger_time = None

        # === Trigger stop when threshold crossed ===
        if force > self.threshold_g:
            self.get_logger().warn(
                f'Force {force:.1f} g > {self.threshold_g:.1f} g → EMERGENCY STOP!'
            )

            # Publish 1-shot stop pulse
            self.stop_pub.publish(Bool(data=True))

            # Start cooldown
            self.last_trigger_time = now

    def get_next_blocks(self):
        """
        Decide the next block targets for push, pull, and place.
        """
        # for now, cycle the same blocks every loop:
        return ('block22f', 'block22b', 'block72b')

def main():
    rclpy.init()
    node = Brain()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
