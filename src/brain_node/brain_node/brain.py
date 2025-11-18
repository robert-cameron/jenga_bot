#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, Bool


class ForceStopper(Node):
    def __init__(self):
        super().__init__('force_stopper')

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

        self.get_logger().info(
            f'force_stopper up. Watching /prongs/force_g > {self.threshold_g:.1f} g.'
        )

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


def main():
    rclpy.init()
    node = ForceStopper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
