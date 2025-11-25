#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Bool, String

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

        # Track blocks that were too hard to push
        self._immovable_lock = threading.Lock()
        # set of (row_num, pos) tuples
        self._immovable_blocks = set()

        # Track which block we are currently pushing
        # tuple (row_num, pos) or None
        self._current_push_block = None

        # Last chosen block from get_next_blocks (row_num, pos)
        self._last_chosen_block = None

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

        self.prongs_mode_pub = self.create_publisher(
            String,
            '/prongs/mode',
            10
        )

        # startup action client
        self.manipulation_client = ActionClient(
            self,
            Manipulation,
            '/manipulation_action'
        )
        # start the action chain once the executor is spinning.
        self.sequence_timer = self.create_timer(5, self._start_sequence)
        self.create_timer(0.1, self._try_close_prongs)
        self._prongs_closed = False
        self.sequence_thread = None

        self.get_logger().info(
            f'force_stopper up. Watching /prongs/force_g > {self.threshold_g:.1f} g.'
        )

    # ------------------------------------------------------------------
    # Sequence and actions
    # ------------------------------------------------------------------
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

            self.get_logger().info(f'Starting Sequence for {push_tf}, {pull_tf} {place_tf}')

            moves = [
                ('push_move', push_tf),
                ('pull_move', pull_tf),
                ('place_move', place_tf),
            ]

            for action_type, tf in moves:
                if not tf:
                    self.get_logger().error(f'TF frame missing for {action_type}; aborting sequence.')
                    return

                # If this is the push, remember which block we are trying
                if action_type == 'push_move':
                    with self._immovable_lock:
                        self._current_push_block = self._last_chosen_block

                self.get_logger().info(f'Sending {action_type} targeting {tf}...')
                success = self._send_goal_and_wait(action_type, tf)

                # Clear current_push_block after push completes
                if action_type == 'push_move':
                    with self._immovable_lock:
                        self._current_push_block = None

                if not success:
                    self.get_logger().error(f'{action_type} failed; stopping sequence.')
                    if action_type != 'push_move':
                        self.get_logger().error('cannot recover from here: killing loop.')
                        return # pull or place has failed. cannot recover from here 
                    else:
                        break

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
                self.get_logger().debug(f"On result {result}")
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
            self.get_logger().warn("Not Accepted")
            return False

        # wait for result
        goal_result.wait()
        self.get_logger().warn(f"Goal Result {outcome['succeeded']}")

        return outcome['succeeded']

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
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

            # Mark the currently pushed block as immovable (if any)
            with self._immovable_lock:
                if self._current_push_block is not None:
                    row_num, pos = self._current_push_block
                    self._immovable_blocks.add(self._current_push_block)
                    self.get_logger().warn(
                        f'Marking block at row {row_num}, pos {pos} as IMMOVABLE due to high force.'
                    )

    def on_tower(self, msg: Tower):
        """Store the latest tower state from computer vision."""
        with self._tower_lock:
            self._latest_tower = msg

    # ------------------------------------------------------------------
    # Jenga logic
    # ------------------------------------------------------------------
    def get_next_blocks(self):
        """
        Decide the next block targets for push, pull, and place.

        Rules:
        - Start from the second bottom row (index 1).
        - Only consider rows with:
            * 3 blocks → normally push/pull the middle one (pos2),
              but if that block is marked IMMOVABLE, pick a side instead.
            * exactly 2 blocks AND the centre is present (pos2 + pos1 or pos2 + pos3).
              If centre is IMMOVABLE, fall back to the side.
        - Skip rows with:
            * 1 block, OR
            * 2 blocks but no centre (only pos1 + pos3).
        """

        # Grab latest tower snapshot
        with self._tower_lock:
            tower = self._latest_tower

        if tower is None or not tower.rows:
            self.get_logger().warn('No tower data yet; using hard-coded fallback.')
            self._last_chosen_block = None
            return ('', '', '')

        rows = tower.rows
        num_rows = len(rows)

        # Snapshot immovable set
        with self._immovable_lock:
            immovable = set(self._immovable_blocks)

        chosen_row_idx = None
        chosen_pos = None

        # Start from second bottom row (index 1) and go upwards
        for i in range(1, num_rows):
            row = rows[i]
            row_num = i + 1  # 1-based for naming and immovable keys

            occ = {
                1: bool(row.pos1),
                2: bool(row.pos2),
                3: bool(row.pos3),
            }
            count = sum(occ.values())

            # Skip empty rows
            if count == 0:
                continue

            # --- Apply your rules with immovable check ---
            if count == 3:
                # Ideal order: centre, then sides — but skip immovable ones
                for p in [2, 1, 3]:
                    if occ[p] and (row_num, p) not in immovable:
                        chosen_row_idx = i
                        chosen_pos = p
                        break
                if chosen_row_idx is not None:
                    break

            elif count == 2:
                # Only valid if centre is present
                if not occ[2]:
                    # two blocks but no centre: skip this row
                    continue

                # Blocks are centre + one side: try centre unless IMMOVABLE,
                # then try the side, else skip
                side = 1 if occ[1] else 3
                for p in [2, side]:
                    if (row_num, p) not in immovable:
                        chosen_row_idx = i
                        chosen_pos = p
                        break
                if chosen_row_idx is not None:
                    break

            else:
                # count == 1 → skip this row entirely (don't touch single-block rows)
                continue

        if chosen_row_idx is None or chosen_pos is None:
            self.get_logger().warn(
                'No suitable row found (respecting immovable blocks); using fallback.'
            )
            self._last_chosen_block = None
            return ('', '', '')

        row_num = chosen_row_idx + 1  # 1-based

        # Save last chosen logical block for tracking during push
        self._last_chosen_block = (row_num, chosen_pos)

        # Push/pull frames for the chosen block
        push_tf = f'block{row_num}{chosen_pos}b'
        pull_tf = f'block{row_num}{chosen_pos}f'

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

        place_tf = f'block{place_row}{place_pos}f'

        self.get_logger().info(
            f'\n'
            f'=== JENGA MOVE DECISION ===\n'
            f'  → Remove: row={row_num}, position={chosen_pos}\n'
            f'  → push_tf:  {push_tf}\n'
            f'  → pull_tf:  {pull_tf}\n'
            f'  → place_tf: {place_tf}\n'
            f'  → Immovable blocks: {sorted(list(immovable))}\n'
            f'============================'
        )

        return (push_tf, pull_tf, place_tf)
    
    def _try_close_prongs(self):
        # If already done, stop checking
        if self._prongs_closed:
            return

        count = self.prongs_mode_pub.get_subscription_count()
        if count == 0:
            # No subscriber yet → wait
            self.get_logger().info("Waiting for /prongs/mode subscriber...")
            return

        # Subscriber exists → publish now
        msg = String()
        msg.data = "cf"
        self.prongs_mode_pub.publish(msg)
        self._prongs_closed = True
        self.get_logger().info("Prongs closed after subscriber detected.")

        # Kill the timer by returning without rescheduling (ROS2 will stop it)
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
