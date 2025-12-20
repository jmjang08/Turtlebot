#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class Nav2ReadyWaiter(Node):
    """
    Helper node that waits until the /robotN/navigate_to_pose action server (Nav2) 
    is active and then terminates immediately.
    """
    def __init__(self):
        super().__init__('nav2_ready_waiter')

        # Parameters: robot_id (1 or 3), timeout (seconds)
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('timeout', 300.0)  # Wait up to 5 minutes by default

        robot_id_param = self.get_parameter('robot_id').value
        timeout_param = self.get_parameter('timeout').value

        # Handle cases where robot_id might be passed as a string from Launch
        try:
            self.robot_id = int(robot_id_param)
        except Exception:
            self.robot_id = 1

        try:
            self.timeout = float(timeout_param)
        except Exception:
            self.timeout = 300.0

        self.action_name = f'/robot{self.robot_id}/navigate_to_pose'
        self._client = ActionClient(self, NavigateToPose, self.action_name)

        self.get_logger().info(
            f'Nav2ReadyWaiter started: robot_id={self.robot_id}, '
            f'waiting for action server [{self.action_name}], '
            f'timeout={self.timeout}s'
        )

    def wait_for_nav2(self) -> bool:
        start = time.time()
        # Check for action server availability at 1-second intervals
        while rclpy.ok():
            if self._client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info(
                    f'Nav2 NavigateToPose action server is available at [{self.action_name}]'
                )
                return True

            elapsed = time.time() - start
            self.get_logger().info(
                f'Waiting for Nav2 action server [{self.action_name}]... '
                f'elapsed={elapsed:.1f}s'
            )
            if elapsed > self.timeout:
                self.get_logger().error(
                    f'Timeout ({self.timeout}s) waiting for Nav2 action server [{self.action_name}]'
                )
                return False
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ReadyWaiter()
    try:
        ok = node.wait_for_nav2()
    except KeyboardInterrupt:
        ok = False
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
