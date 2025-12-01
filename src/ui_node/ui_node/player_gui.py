#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32


class PlayerGUI(Node):
    """
    Simple window-based UI for the Jenga game.

    - "Start / Next Move" button publishes Bool(True) on /ui/player_done.
    - Prongs buttons send mode commands on /prongs/mode: 'o', 'cp', 'cf'.
    - Subscribes to /prongs/force_g and exposes latest value via self.current_force_g.
    """

    def __init__(self):
        super().__init__('player_gui')

        # ----- Publishers -----
        self.done_pub = self.create_publisher(Bool, '/ui/player_done', 10)
        self.prongs_mode_pub = self.create_publisher(String, '/prongs/mode', 10)

        # ----- Force subscriber -----
        self.current_force_g = 0.0
        self.force_sub = self.create_subscription(
            Float32,
            '/prongs/force_g',
            self.force_callback,
            10
        )

        self.get_logger().info("PlayerGUI node initialised.")

    # ------------- ROS callbacks -------------

    def force_callback(self, msg: Float32):
        """Store the latest force in grams."""
        self.current_force_g = msg.data

    # ------------- ROS publish helpers -------------

    def send_player_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        self.get_logger().info("Sent /ui/player_done: True")

    def send_prongs_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.prongs_mode_pub.publish(msg)
        self.get_logger().info(f'Sent /prongs/mode: "{mode}"')


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
    root.geometry("420x320")

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
            "1. Click “Start / Next Move” when you want the robot to move.\n"
            "2. After the robot finishes and you've done your turn,\n"
            "   click it again for the next move."
        ),
        justify="left",
        wraplength=380
    )
    info_label.pack(pady=(0, 10))

    status_var = tk.StringVar(
        value="Click “Start / Next Move” when you’re ready."
    )

    status_label = ttk.Label(
        main_frame,
        textvariable=status_var,
        foreground="grey"
    )
    status_label.pack(pady=(0, 10))

    # --- Player start/next button ---

    def on_start_next():
        node.send_player_done()
        status_var.set(
            "Signal sent. Wait for the robot to finish,\n"
            "then click again for the next move."
        )

    start_button = ttk.Button(
        main_frame,
        text="Start / Next Move",
        command=on_start_next
    )
    start_button.pack(pady=(0, 15), ipadx=10, ipady=5)

    # --- Prongs control buttons (use /prongs/mode: 'o', 'cp', 'cf') ---

    prongs_frame = ttk.LabelFrame(main_frame, text="Prongs Control")
    prongs_frame.pack(pady=(0, 10), fill="x")

    def make_mode_button(text: str, mode: str, col: int):
        btn = ttk.Button(
            prongs_frame,
            text=text,
            command=lambda: node.send_prongs_mode(mode)
        )
        btn.grid(row=0, column=col, padx=5, pady=5, ipadx=5, ipady=3)
        return btn

    # Map to your CLI commands:
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'o'"
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cp'"
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cf'"
    make_mode_button("Open (o)",        "o",  0)
    make_mode_button("Grip Block (cp)", "cp", 1)
    make_mode_button("Close Full (cf)","cf", 2)

    # --- Force display ---

    force_frame = ttk.LabelFrame(main_frame, text="Force (g)")
    force_frame.pack(pady=(10, 0), fill="x")

    ttk.Label(force_frame, text="Current force:").grid(
        row=0, column=0, padx=5, pady=5, sticky="w"
    )

    force_value_var = tk.StringVar(value="0.0 g")

    # Use tk.Label (not ttk) so we can change foreground colour easily
    force_value_label = tk.Label(
        force_frame,
        textvariable=force_value_var,
        font=("Helvetica", 14, "bold"),
        fg="green"
    )
    force_value_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")

    THRESHOLD_G = 80.0

    def refresh_force_label():
        # Read from node (updated in ROS callback thread)
        value = node.current_force_g
        force_value_var.set(f"{value:.1f} g")

        if value > THRESHOLD_G:
            force_value_label.config(fg="red")
        else:
            force_value_label.config(fg="green")

        # Schedule next update
        root.after(100, refresh_force_label)  # every 100 ms

    # Kick off periodic UI update
    refresh_force_label()

    # Handle closing the window
    def on_close():
        node.get_logger().info("Closing PlayerGUI, shutting down ROS 2…")
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    root.mainloop()

    spin_thread.join(timeout=1.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
