#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose, PoseArray
from manipulation.action import Manipulation


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # === Parameters ===
        self.declare_parameter('open_gap_mm', 30.0)       # demo open
        self.declare_parameter('close_gap_mm', 5.0)       # demo close
        self.declare_parameter('push_action', 'push_move')# MoveIt action to push
        self.declare_parameter('result_timeout_s', 10.0)  # action result timeout
        self.declare_parameter('pick_policy', 'highest_z')

        self.open_gap = float(self.get_parameter('open_gap_mm').value)
        self.close_gap = float(self.get_parameter('close_gap_mm').value)
        self.push_action = self.get_parameter('push_action').get_parameter_value().string_value
        self.result_timeout_s = float(self.get_parameter('result_timeout_s').value)
        self.pick_policy = self.get_parameter('pick_policy').get_parameter_value().string_value

        # === I/O ===
        self.prongs_cmd_pub = self.create_publisher(String, '/prongs/cmd', 10)
        self.create_subscription(Bool,      '/ui/player_done', self.on_start_signal, 10)
        self.create_subscription(PoseArray, '/vision/blocks',  self.on_blocks,       10)
        self.create_subscription(Float32,   '/prongs/force_g', self.on_force,        10)  # optional

        self.manip_client = ActionClient(self, Manipulation, 'manipulation_action')

        # === State ===
        self.running = False           # set by SPACE once
        self.busy = False              # sequence in progress
        self.last_force = 0.0
        self.latest_pose = None
        self.latest_pose_seq = 0
        self.consumed_seq = -1
        self.last_start_stamp = self.get_clock().now() - Duration(seconds=1.0)

        self.create_timer(0.2, self.tick)
        self.log('Brain up. Press SPACE (UI) to start. Waiting…')

    # ---------- helpers ----------
    def log(self, s: str):
        self.get_logger().info(s)

    def send_prongs(self, s: str):
        self.prongs_cmd_pub.publish(String(data=s))
        self.get_logger().info(f'/prongs/cmd → "{s}"')

    # ---------- subs ----------
    def on_start_signal(self, msg: Bool):
        now = self.get_clock().now()
        if msg.data and (now - self.last_start_stamp) > Duration(seconds=0.2):
            self.last_start_stamp = now
            if not self.running:
                self.running = True
                self.log('RUNNING mode enabled (SPACE). Will demo gripper then push.')
            else:
                self.log('Already running; SPACE ignored.')

    def on_blocks(self, msg: PoseArray):
        if not msg.poses:
            return
        if self.pick_policy == 'highest_z':
            best = max(msg.poses, key=lambda p: p.position.z)
        else:
            best = msg.poses[0]
        self.latest_pose = best
        self.latest_pose_seq += 1

    def on_force(self, msg: Float32):
        self.last_force = msg.data

    # ---------- main loop ----------
    def tick(self):
        if not self.running or self.busy:
            return
        if self.latest_pose is None or self.latest_pose_seq == self.consumed_seq:
            # Waiting for a fresh pose from vision
            return

        target = self.latest_pose
        seq_id = self.latest_pose_seq
        self.busy = True
        self.execute_sequence(target, seq_id)

    # ---------- sequence: demo gripper → push ----------
    def execute_sequence(self, target_pose: Pose, seq_id: int):
        # Quick demo: open then close before any MoveIt call
        self.send_prongs(f'gap {self.open_gap:.2f}')
        self.create_timer(0.30, lambda: self.send_prongs(f'gap {self.close_gap:.2f}'))
        self.create_timer(0.55, lambda: self._push_target(target_pose, seq_id))  # small pause to finish close

    def _push_target(self, target_pose: Pose, seq_id: int):
        self.log(f'Pushing via "{self.push_action}" to z={target_pose.position.z:.3f}')
        self._send_manipulation(self.push_action, target_pose,
                                on_done=lambda ok: self._finish_cycle(ok, seq_id))

    def _finish_cycle(self, _ok: bool, seq_id: int):
        self.consumed_seq = seq_id
        self.busy = False
        self.log(f'Push complete. Force ≈ {self.last_force:.1f} g. Waiting for next coords…')

    # ---------- action helper ----------
    def _send_manipulation(self, action_type: str, pose: Pose, on_done):
        if not self.manip_client.wait_for_server(timeout_sec=0.5):
            self.log('Manipulation server not available.')
            on_done(False)
            return

        goal = Manipulation.Goal()
        goal.action_type = action_type
        goal.block_pose = pose

        def _goal_response_cb(fut):
            handle = fut.result()
            if not handle.accepted:
                self.log('Manipulation goal rejected.')
                on_done(False); return

            res_fut = handle.get_result_async()

            # watchdog timeout
            timed_out = {'hit': False}
            def _watchdog():
                if not timed_out['hit']:
                    timed_out['hit'] = True
                    self.log(f'Manipulation "{action_type}" timed out.')
                    on_done(False)
            self.create_timer(self.result_timeout_s, _watchdog)

            def _res_cb(rf):
                if timed_out['hit']:
                    return
                try:
                    res = rf.result()
                    ok = bool(res.result.result)
                    self.log(f'Manipulation "{action_type}" result: {ok}')
                    on_done(ok)
                except Exception as e:
                    self.log(f'Manipulation result error: {e}')
                    on_done(False)

            res_fut.add_done_callback(_res_cb)

        try:
            self.manip_client.send_goal_async(goal).add_done_callback(_goal_response_cb)
        except Exception as e:
            self.log(f'Failed to send manipulation goal: {e}')
            on_done(False)


def main():
    rclpy.init()
    node = Brain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
