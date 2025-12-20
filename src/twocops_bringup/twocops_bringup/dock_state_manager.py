#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.action import Undock


class DockStateManager(Node):
    """
    - Determines the initial state based on the first dock_status received:
      - If initially is_docked == True  -> Execute UNDOCK action.
      - If initially is_docked == False -> Log a warning and terminate.
    """
    def __init__(self):
        super().__init__('dock_state_manager')

        # Parameters: robot_id (e.g., 1 -> /robot1)
        self.declare_parameter('robot_id', 1)
        robot_id = self.get_parameter('robot_id').value

        if isinstance(robot_id, int):
            ns = f'/robot{robot_id}'
        else:
            ns = f'/{robot_id.lstrip("/")}'  # Allow string like "robot1"

        self.ns = ns

        # State variables
        self.is_docked = None
        self.initial_state_checked = False  # Track if the initial state has been evaluated
        self._undock_sent = False
        self.done = False

        # Action Client (/robotN/undock)
        self.undock_client = ActionClient(self, Undock, f'{self.ns}/undock')

        # Subscribe to dock status (/robotN/dock_status)
        self.create_subscription(
            DockStatus,
            f'{self.ns}/dock_status',
            self.dock_status_callback,
            10,
        )

        self.get_logger().info(
            f"DockStateManager initialized (ns={self.ns}) - waiting for dock_status..."
        )

    # -----------------------------
    # DockStatus Callback
    # -----------------------------
    def dock_status_callback(self, msg: DockStatus):
        if self.done:
            return

        self.is_docked = msg.is_docked
        self.get_logger().info(f"Dock status received: is_docked={self.is_docked}")

        # Evaluate initial state only on the first message
        if not self.initial_state_checked:
            self.initial_state_checked = True

            if self.is_docked:
                # Initially docked -> proceed with undocking
                self.get_logger().info("Robot is docked at startup -> sending UNDOCK goal")
                self.send_undock_goal()
            else:
                # Already undocked -> log warning and exit
                self.get_logger().warn(
                    "Robot is already undocked at startup. "
                    "Please start the scenario from the DOCK state."
                )
                self.done = True

        else:
            # Subsequent dock_status messages are logged for debugging
            self.get_logger().debug(
                f"[DEBUG] Subsequent dock_status: is_docked={self.is_docked}"
            )

    # -----------------------------
    # Undock Action Logic
    # -----------------------------
    def send_undock_goal(self):
        self._undock_sent = True

        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Undock action server not available after 10 seconds."
            )
            self.done = True
            return

        goal_msg = Undock.Goal()
        send_future = self.undock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.undock_feedback_callback,
        )
        send_future.add_done_callback(self.undock_goal_response_callback)

    def undock_feedback_callback(self, feedback_msg):
        # Optional: Log feedback if needed
        pass

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected")
            self.done = True
            return

        self.get_logger().info("Undock goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Undock succeeded.")
        else:
            self.get_logger().warn(f"Undock finished with status: {status}")

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = DockStateManager()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("DockStateManager finished. Shutting down...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
