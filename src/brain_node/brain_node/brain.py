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
from tower_interfaces.msg import Tower

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

        # Get Jenga Blocks from Vision
        self.tower_sub = self.create_subscription(
            Tower,
            '/vision/tower',      
            self.on_tower,
            10,
        )

        self._tower_lock = threading.Lock()
        self._latest_tower = None

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
    def on_tower(self, msg: Tower):
        """Store the latest tower state from computer vision."""
        with self._tower_lock:
            self._latest_tower = msg

    def get_next_blocks(self):
        """
        Decide the next block targets for push, pull, and place.

        Rules:
        - Start from the second bottom row (index 1).
        - Only consider rows with:
            * 3 blocks  → push/pull the middle one (pos2), OR
            * exactly 2 blocks AND the centre is present (pos2 + pos1 or pos2 + pos3).
        - Skip rows with:
            * 1 block, OR
            * 2 blocks but no centre (only pos1 + pos3).
        """

        # Grab latest tower snapshot
        with self._tower_lock:
            tower = self._latest_tower

        if tower is None or not tower.rows:
            self.get_logger().warn('No tower data yet; using hard-coded fallback.')
            return ('block22f', 'block22b', 'block72b')

        rows = tower.rows
        num_rows = len(rows)

        chosen_row_idx = None
        chosen_pos = None

        # Start from second bottom row (index 1) and go upwards
        for i in range(1, num_rows):
            row = rows[i]

            occ = {
                1: bool(row.pos1),
                2: bool(row.pos2),
                3: bool(row.pos3),
            }
            count = sum(occ.values())

            # Skip empty rows
            if count == 0:
                continue

            # --- Apply your rules ---
            if count == 3:
                # 3 blocks → use centre
                chosen_row_idx = i
                chosen_pos = 2
                break

            if count == 2:
                # Only valid if centre is present (pos2 + pos1 or pos2 + pos3)
                if occ[2]:
                    chosen_row_idx = i
                    chosen_pos = 2
                    break
                else:
                    # Two blocks but no centre (pos1+pos3) → skip this row
                    continue

            # count == 1 → skip this row entirely
            # (don't remove from rows with a single block)
            if count == 1:
                continue

        if chosen_row_idx is None or chosen_pos is None:
            self.get_logger().warn(
                'No suitable row found (needs 3 blocks or 2 with centre); using fallback.'
            )
            return ('block22f', 'block22b', 'block72b')

        row_num = chosen_row_idx + 1  # rows are 1-based in frame names

        # Push/pull frames for the chosen block
        push_tf = f'block{row_num}{chosen_pos}f'
        pull_tf = f'block{row_num}{chosen_pos}b'

        # --- Decide where to place the block ---
        # If the top row has 3 blocks → place ABOVE it (new row, centre back).
        # If the top row has <3 blocks → place in the GAP on that row
        # (prefer centre, then left, then right), back face.

        top_row = rows[num_rows - 1]
        top_row_num = num_rows

        top_occ = {
            1: bool(top_row.pos1),
            2: bool(top_row.pos2),
            3: bool(top_row.pos3),
        }

        if sum(top_occ.values()) == 3:
            # Full top row → create a new row above, centre back
            place_row = top_row_num + 1
            place_pos = 2
        else:
            # There is at least one gap → fill it on the current top row
            if not top_occ[2]:
                place_row = top_row_num
                place_pos = 2
            elif not top_occ[1]:
                place_row = top_row_num
                place_pos = 1
            else:
                place_row = top_row_num
                place_pos = 3

        place_tf = f'block{place_row}{place_pos}b'

        self.get_logger().info(
            f'Chosen row {row_num}, pos {chosen_pos}: '
            f'push_tf={push_tf}, pull_tf={pull_tf}, place_tf={place_tf}'
        )

        return (push_tf, pull_tf, place_tf)



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
