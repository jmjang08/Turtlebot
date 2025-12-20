#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Dock, Undock


class DockingNode(Node):
    """
    A node that calls /robotN/dock or /robotN/undock actions once,
    waits for the result, and then terminates.
    - Parameters:
      - robot_id (int or str, e.g., 1 -> /robot1)
      - command: 'dock' or 'undock'
    """
    def __init__(self):
        super().__init__('docking_node')

        # Declare parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('command', 'undock')

        robot_id = self.get_parameter('robot_id').value
        command = self.get_parameter('command').value

        # Create /robotX namespace
        if isinstance(robot_id, int):
            ns = f'/robot{robot_id}'
        else:
            ns = f'/{robot_id.lstrip("/")}'  # Allow string like "robot1"

        self.ns = ns
        self.command = command

        # Create Action Clients
        self.undock_client = ActionClient(self, Undock, f'{self.ns}/undock')
        self.dock_client = ActionClient(self, Dock, f'{self.ns}/dock')

        self._active_goal_handle = None

        self.get_logger().info(
            f"DockingNode initialized: ns={self.ns}, command={self.command}"
        )

        # Start the requested action
        if self.command == 'undock':
            self.send_undock_goal()
        elif self.command == 'dock':
            self.send_dock_goal()
        else:
            self.get_logger().error(f"Unknown command: {self.command}")
            self._shutdown()

    # --------------------
    # Undock Action
    # --------------------
    def send_undock_goal(self):
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Undock action server not available.")
            self._shutdown()
            return

        self.get_logger().info("Sending Undock goal...")
        goal_msg = Undock.Goal()
        send_future = self.undock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.undock_feedback_callback
        )
        send_future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected")
            self._shutdown()
            return

        self.get_logger().info("Undock goal accepted")
        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_feedback_callback(self, feedback_msg):
        pass

    def undock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Undock succeeded.")
        else:
            self.get_logger().warn(f"Undock finished with status: {status}")

        self._active_goal_handle = None
        self._shutdown()

    # --------------------
    # Dock Action
    # --------------------
    def send_dock_goal(self):
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock action server not available.")
            self._shutdown()
            return

        self.get_logger().info("Sending Dock goal...")
        goal_msg = Dock.Goal()
        send_future = self.dock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.dock_feedback_callback
        )
        send_future.add_done_callback(self.dock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected")
            self._shutdown()
            return

        self.get_logger().info("Dock goal accepted")
        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_result_callback)

    def dock_feedback_callback(self, feedback_msg):
        pass

    def dock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Dock succeeded.")
        else:
            self.get_logger().warn(f"Dock finished with status: {status}")

        self._active_goal_handle = None
        self._shutdown()

    # --------------------
    # Common Shutdown Process
    # --------------------
    def _shutdown(self):
        self.get_logger().info("DockingNode finished. Shutting down...")
        # Direct shutdown of rclpy
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass

if __name__ == "__main__":
    main()
