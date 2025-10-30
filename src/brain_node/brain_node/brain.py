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
        self.declare_parameter('open_gap_mm', 30.0)        # wide approach gap
        self.declare_parameter('approach_gap_mm', 18.0)    # near block width
        self.declare_parameter('close_gap_mm', 5.0)        # “grip” gap
        self.declare_parameter('approach_action', 'linear_move')
        self.declare_parameter('lift_action', 'free_move')
        self.declare_parameter('lift_dz', 0.05)            # 5 cm lift
        self.declare_parameter('result_timeout_s', 10.0)   # action result timeout

        self.open_gap = float(self.get_parameter('open_gap_mm').value)
        self.approach_gap = float(self.get_parameter('approach_gap_mm').value)
        self.close_gap = float(self.get_parameter('close_gap_mm').value)
        self.approach_action = self.get_parameter('approach_action').get_parameter_value().string_value
        self.lift_action = self.get_parameter('lift_action').get_parameter_value().string_value
        self.lift_dz = float(self.get_parameter('lift_dz').value)
        self.result_timeout_s = float(self.get_parameter('result_timeout_s').value)

        # === I/O ===
        self.prongs_cmd_pub = self.create_publisher(String, '/prongs/cmd', 10)
        self.create_subscription(Bool, '/ui/player_done', self.on_player_done, 10)
        self.create_subscription(PoseArray, '/vision/blocks', self.on_blocks, 10)
        self.create_subscription(Float32, '/prongs/force_g', self.on_force, 10)  # optional feedback

        # Action client
        self.manip_client = ActionClient(self, Manipulation, 'manipulation_action')

        # === State ===
        self.turn = 'PLAYER'
        self.player_done_latch = False
        self.blocks = []
        self.busy = False
        self.last_force = 0.0
        self.last_player_done_stamp = self.get_clock().now() - Duration(seconds=1.0)

        # main loop
        self.create_timer(0.2, self.tick)
        self.log('Brain up. Waiting for player…')

    # ------------- helpers -------------
    def log(self, s: str):
        self.get_logger().info(s)

    def send_prongs(self, s: str):
        self.prongs_cmd_pub.publish(String(data=s))
        self.log(f'/prongs/cmd → "{s}"')

    # ------------- subscribers -------------
    def on_player_done(self, msg: Bool):
        # Debounce: only accept rising-true edges >200ms apart
        now = self.get_clock().now()
        if msg.data and (now - self.last_player_done_stamp) > Duration(seconds=0.2):
            self.player_done_latch = True
            self.last_player_done_stamp = now
            self.log('Player indicates done.')

    def on_blocks(self, msg: PoseArray):
        self.blocks = list(msg.poses)

    def on_force(self, msg: Float32):
        self.last_force = msg.data

    # ------------- FSM -------------
    def tick(self):
        if self.busy:
            return

        if self.turn == 'PLAYER':
            if self.player_done_latch:
                self.player_done_latch = False
                self.turn = 'ROBOT'
                self.log('Robot turn.')
        elif self.turn == 'ROBOT':
            if not self.blocks:
                self.log('No blocks detected; waiting…')
                return
            target = max(self.blocks, key=lambda p: p.position.z)  # choose highest Z
            self.busy = True
            self.execute_sequence(target)
        else:
            self.turn = 'PLAYER'

    # ------------- main sequence using Manipulation + prongs -------------
    def execute_sequence(self, target_pose: Pose):
        # 1) Open wide, then approach width (simple pacing with small timers)
        self.send_prongs(f'gap {self.open_gap:.2f}')
        self.create_timer(0.25, lambda: self.send_prongs(f'gap {self.approach_gap:.2f}'))

        # 2) Manipulation approach
        self.log(f'Approach via action="{self.approach_action}" to z={target_pose.position.z:.3f}')
        self._send_manipulation(self.approach_action, target_pose,
                                on_done=lambda ok: self._after_approach(ok, target_pose))

    def _after_approach(self, ok: bool, target_pose: Pose):
        if not ok:
            self.log('Approach failed. Handing back to player.')
            self.busy = False
            self.turn = 'PLAYER'
            return

        # 3) Close to grip; ask for a force read to log
        self.send_prongs(f'gap {self.close_gap:.2f}')
        self.create_timer(0.15, lambda: self.send_prongs('force'))

        # 4) Lift slightly using a second Manipulation goal
        lifted = Pose()
        lifted.position.x = target_pose.position.x
        lifted.position.y = target_pose.position.y
        lifted.position.z = target_pose.position.z + self.lift_dz
        lifted.orientation = target_pose.orientation

        self.log('Lift object slightly.')
        self._send_manipulation(self.lift_action, lifted, on_done=self._finish_turn)

    def _finish_turn(self, _ok: bool):
        # Regardless of lift result, finish the robot turn for now.
        self.busy = False
        self.turn = 'PLAYER'
        self.log(f'Robot turn complete. Last force ≈ {self.last_force:.1f} g. Player turn.')

    # ------------- action helper with timeout & errors -------------
    def _send_manipulation(self, action_type: str, pose: Pose, on_done):
        # Ensure server is up (non-blocking retries)
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
                on_done(False)
                return

            res_fut = handle.get_result_async()

            def _res_cb(rf):
                try:
                    res = rf.result()
                    ok = bool(res.result.result)
                    self.log(f'Manipulation "{action_type}" result: {ok}')
                    on_done(ok)
                except Exception as e:
                    self.log(f'Manipulation result error: {e}')
                    on_done(False)

            # Add a watchdog timeout
            self.create_timer(self.result_timeout_s,
                              lambda: (self.log(f'Manipulation "{action_type}" timed out'), on_done(False)))
            res_fut.add_done_callback(_res_cb)

        try:
            send_fut = self.manip_client.send_goal_async(goal)
            send_fut.add_done_callback(_goal_response_cb)
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
