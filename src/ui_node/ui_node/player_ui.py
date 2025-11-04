#!/usr/bin/env python3
import sys, select, termios, tty, atexit
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class PlayerUI(Node):
    def __init__(self):
        super().__init__('player_ui')

        # Publishers
        self.done_pub   = self.create_publisher(Bool,   '/ui/player_done', 10)
        # (optional) quick hook to prongs if you want shortcuts:
        self.prongs_pub = self.create_publisher(String, '/prongs/cmd',     10)

        # Put terminal in raw mode so SPACE is captured instantly
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        atexit.register(self._restore_tty)

        # Poll keyboard ~20 Hz
        self.timer = self.create_timer(0.05, self._poll_keys)

        self.get_logger().info(
            "UI ready.\n"
            "  SPACE → publish /ui/player_done True\n"
            "  o → send 'open 30'   (to /prongs/cmd)\n"
            "  c → send 'close 5'   (to /prongs/cmd)\n"
            "  g → send 'gap 18'    (to /prongs/cmd)\n"
            "  q → quit"
        )

    def _restore_tty(self):
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        except Exception:
            pass

    def _poll_keys(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == ' ':
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info("→ Player done (SPACE)")
            elif ch == 'o':
                self.prongs_pub.publish(String(data='open 30'))
                self.get_logger().info("→ /prongs/cmd: open 30")
            elif ch == 'c':
                self.prongs_pub.publish(String(data='close 5'))
                self.get_logger().info("→ /prongs/cmd: close 5")
            elif ch == 'g':
                self.prongs_pub.publish(String(data='gap 18'))
                self.get_logger().info("→ /prongs/cmd: gap 18")
            elif ch in ('q', 'Q'):
                self.get_logger().info("Quitting UI…")
                rclpy.shutdown()

def main():
    rclpy.init()
    node = PlayerUI()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
