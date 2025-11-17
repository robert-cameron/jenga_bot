#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class ForceStopper(Node):
    def __init__(self):
        super().__init__('force_stopper')

        # === Parameters ===
        self.declare_parameter('threshold_g', 80.0)
        self.threshold_g = float(self.get_parameter('threshold_g').value)

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

        self.tripped = False
        self.get_logger().info(
            f'force_stopper up. Watching /prongs/force_g > {self.threshold_g:.1f} g.'
        )

    def on_force(self, msg: Float32):
        force = msg.data
        # If force exceeds threshold, publish stop
        if force > self.threshold_g:
            if not self.tripped:
                self.get_logger().warn(
                    f'Force {force:.1f} g > {self.threshold_g:.1f} g → EMERGENCY STOP!'
                )
                self.tripped = True
            # Publish stop (latched behaviour by logic, not by QoS)
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
        else:
            # Optional: reset latch when back below threshold
            if self.tripped and force < self.threshold_g * 0.8:
                self.get_logger().info(
                    f'Force dropped to {force:.1f} g, resetting stop latch.'
                )
                self.tripped = False


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
