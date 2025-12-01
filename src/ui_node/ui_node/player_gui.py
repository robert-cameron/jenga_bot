#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PlayerGUI(Node):
    """
    Simple window-based UI for the Jenga game.

    - "Start / Next Move" button publishes Bool(True) on /ui/player_done.
    - Optional buttons for gripper control via /prongs/cmd.
    """

    def __init__(self):
        super().__init__('player_gui')

        # ROS publishers
        self.done_pub = self.create_publisher(Bool, '/ui/player_done', 10)
        self.prongs_pub = self.create_publisher(String, '/prongs/cmd', 10)

        self.get_logger().info("PlayerGUI node initialised.")

    # ------------- ROS publish helpers -------------

    def send_player_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        self.get_logger().info("Sent /ui/player_done: True")

    def send_prongs_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.prongs_pub.publish(msg)
        self.get_logger().info(f'Sent /prongs/cmd: "{cmd}"')


def main():
    rclpy.init()
    node = PlayerGUI()

    # Spin ROS in a background thread so the Tk window can own the main thread
    def ros_spin():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # ------------- Tkinter UI -------------

    root = tk.Tk()
    root.title("JengaBot – Player Console")

    # Make the window a reasonable size
    root.geometry("400x250")

    # Main frame
    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill="both", expand=True)

    title_label = ttk.Label(
        main_frame,
        text="JengaBot – Player Console",
        font=("Helvetica", 16, "bold")
    )
    title_label.pack(pady=(0, 10))

    info_label = ttk.Label(
        main_frame,
        text=(
            "1. When you are ready for the robot to make a move,\n"
            "   click the button below.\n"
            "2. After the robot finishes and you've finished your turn,\n"
            "   click it again for the next move."
        ),
        justify="left",
        wraplength=360
    )
    info_label.pack(pady=(0, 15))

    status_var = tk.StringVar(
        value="Click “Start / Next Move” when you’re ready."
    )

    status_label = ttk.Label(
        main_frame,
        textvariable=status_var,
        foreground="grey"
    )
    status_label.pack(pady=(0, 10))

    # --- Buttons ---

    def on_start_next():
        node.send_player_done()
        status_var.set("Signal sent. Wait for the robot to finish, "
                       "then click again for the next move.")

    start_button = ttk.Button(
        main_frame,
        text="Start / Next Move",
        command=on_start_next
    )
    start_button.pack(pady=(0, 15), ipadx=10, ipady=5)

    # Optional: simple gripper controls
    button_frame = ttk.Frame(main_frame)
    button_frame.pack(pady=(0, 10))

    def make_prongs_button(text, cmd):
        return ttk.Button(
            button_frame,
            text=text,
            command=lambda: node.send_prongs_cmd(cmd)
        )

    open_btn = make_prongs_button("Open Gripper", "open")
    close_btn = make_prongs_button("Close Gripper", "close")
    force_btn = make_prongs_button("Read Force", "force")

    open_btn.grid(row=0, column=0, padx=5, pady=5)
    close_btn.grid(row=0, column=1, padx=5, pady=5)
    force_btn.grid(row=0, column=2, padx=5, pady=5)

    # Handle closing the window
    def on_close():
        node.get_logger().info("Closing PlayerGUI, shutting down ROS 2…")
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # Start Tk main loop (blocks until window closed)
    root.mainloop()

    # Clean up ROS node when Tk exits
    spin_thread.join(timeout=1.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
