#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32


class PlayerGUI(Node):
    """
    JengaBot player console.

    - "Start / Next Move" publishes Bool(True) on /ui/player_done.
    - Prongs buttons send mode commands on /prongs/mode: 'o', 'cp', 'cf'.
    - Subscribes to /prongs/force_g to show current force.
    - Subscribes to /ui/robot_turn (Bool):
        True  = robot moving → Start button disabled.
        False = human turn  → Start button enabled.
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

        # ----- Robot turn / state -----
        # True  -> robot moving / planning
        # False -> human's turn
        self.robot_turn = False
        self.robot_turn_sub = self.create_subscription(
            Bool,
            '/ui/robot_turn',
            self.robot_turn_callback,
            10
        )

        self.get_logger().info("PlayerGUI node initialised.")

    # ------------- ROS callbacks -------------

    def force_callback(self, msg: Float32):
        """Store the latest force in grams."""
        self.current_force_g = msg.data

    def robot_turn_callback(self, msg: Bool):
        """Update whether it is the robot's turn (robot moving) or player's."""
        self.robot_turn = msg.data

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
    root.geometry("460x360")

    # --- Basic theming ---
    bg_main = "#1f2430"
    bg_panel = "#262b38"
    fg_text = "#f5f7fa"
    fg_subtle = "#a0a4b8"
    accent = "#5c7cfa"

    root.configure(bg=bg_main)

    style = ttk.Style()
    style.theme_use("clam")

    style.configure("Main.TFrame", background=bg_main)
    style.configure("Panel.TLabelframe", background=bg_panel, foreground=fg_text)
    style.configure("Panel.TLabelframe.Label", background=bg_panel, foreground=fg_text)
    style.configure("Title.TLabel", background=bg_main, foreground=fg_text,
                    font=("Helvetica", 18, "bold"))
    style.configure("Body.TLabel", background=bg_main, foreground=fg_subtle)
    style.configure("Status.TLabel", background=bg_main, foreground=fg_subtle)
    style.configure("TButton", padding=6)
    style.map("TButton",
              foreground=[("disabled", "#777777")],
              background=[("active", accent)])

    main_frame = ttk.Frame(root, style="Main.TFrame", padding=20)
    main_frame.pack(fill="both", expand=True)

    title_label = ttk.Label(
        main_frame,
        text="JengaBot – Player Console",
        style="Title.TLabel"
    )
    title_label.pack(pady=(0, 10), anchor="w")

    info_label = ttk.Label(
        main_frame,
        text=(
            "1. When it’s your turn, click “Start / Next Move”.\n"
            "2. While the robot is moving, the button is disabled.\n"
            "3. Once the robot finishes, it becomes your turn again."
        ),
        style="Body.TLabel",
        justify="left",
        wraplength=420
    )
    info_label.pack(pady=(0, 10), anchor="w")

    status_var = tk.StringVar(
        value="Your turn. Click “Start / Next Move” when you’re ready."
    )

    status_label = ttk.Label(
        main_frame,
        textvariable=status_var,
        style="Status.TLabel"
    )
    status_label.pack(pady=(0, 10), anchor="w")

    # --- Player start/next button ---

    button_frame = ttk.Frame(main_frame, style="Main.TFrame")
    button_frame.pack(pady=(0, 15), fill="x")

    start_button = ttk.Button(
        button_frame,
        text="Start / Next Move"
    )
    start_button.pack(ipadx=12, ipady=4)

    def on_start_next():
        # Tell the brain the player is done and robot can move
        node.send_player_done()
        # Optimistically mark as robot turn so UI locks immediately
        node.robot_turn = True
        status_var.set("Robot is moving… please wait.")
        start_button.state(["disabled"])

    start_button.configure(command=on_start_next)

    # --- Prongs control buttons (use /prongs/mode: 'o', 'cp', 'cf') ---

    prongs_frame = ttk.Labelframe(main_frame, text="Prongs Control",
                                  style="Panel.TLabelframe")
    prongs_frame.pack(pady=(0, 10), fill="x")

    # Use a normal tk.Frame inside to override bg
    inner_prongs = tk.Frame(prongs_frame, bg=bg_panel)
    inner_prongs.pack(fill="x", padx=5, pady=5)

    def make_mode_button(text: str, mode: str, col: int):
        btn = ttk.Button(
            inner_prongs,
            text=text,
            command=lambda: node.send_prongs_mode(mode)
        )
        btn.grid(row=0, column=col, padx=5, pady=5, ipadx=5, ipady=3)
        return btn

    # Map to your CLI commands:
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'o'"
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cp'"
    #  ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cf'"
    make_mode_button("Open (o)",         "o",  0)
    make_mode_button("Grip Block (cp)",  "cp", 1)
    make_mode_button("Close Full (cf)",  "cf", 2)

    # --- Force display ---

    force_frame = ttk.Labelframe(main_frame, text="Force (g)",
                                 style="Panel.TLabelframe")
    force_frame.pack(pady=(10, 0), fill="x")

    inner_force = tk.Frame(force_frame, bg=bg_panel)
    inner_force.pack(fill="x", padx=5, pady=5)

    label_force = tk.Label(
        inner_force,
        text="Current force:",
        bg=bg_panel,
        fg=fg_text
    )
    label_force.grid(row=0, column=0, padx=5, pady=5, sticky="w")

    force_value_var = tk.StringVar(value="0.0 g")

    force_value_label = tk.Label(
        inner_force,
        textvariable=force_value_var,
        font=("Helvetica", 16, "bold"),
        bg=bg_panel,
        fg="green"
    )
    force_value_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")

    THRESHOLD_G = 80.0

    def refresh_ui():
        # ----- Force -----
        value = node.current_force_g
        force_value_var.set(f"{value:.1f} g")
        if value > THRESHOLD_G:
            force_value_label.config(fg="red")
        else:
            force_value_label.config(fg="green")

        # ----- Turn / Start button state -----
        if node.robot_turn:
            # Robot is moving or planning
            start_button.state(["disabled"])
            status_var.set("Robot is moving… please wait.")
        else:
            # Human's turn
            start_button.state(["!disabled"])
            # Only overwrite if previous message was "robot moving" or initial
            if "Robot is moving" in status_var.get() or \
               "Your turn" in status_var.get():
                status_var.set(
                    "Your turn. Click “Next Move” when you’re ready."
                )

        # Schedule next update
        root.after(100, refresh_ui)  # every 100 ms

    # Kick off periodic UI update
    refresh_ui()

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
