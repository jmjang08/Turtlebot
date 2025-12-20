#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


class TransitionManager(Node):
    def __init__(self):
        super().__init__('transition_manager')

        # ===============================
        # Parameter load
        # ===============================
        self.declare_parameter('robot_id')
        self.declare_parameter('partner_robot_id')
        
        self.robot_id = self.get_parameter('robot_id').value
        self.partner_robot_id = self.get_parameter('partner_robot_id').value
        
        self.robot_ns = f"/robot{self.robot_id}"
        self.partner_ns = f"/robot{self.partner_robot_id}"
        
        self.get_logger().info(
            f"TransitionManager Started (robot={self.robot_ns}, partner={self.partner_ns})"
        )

        # ===============================
        # State variables
        # ===============================
        self.robot_detected = False
        self.partner_detected = False
        self.last_partner_pose = None
        self.last_redcar_pose = None   # Stores target_state (Red car pose)
        
        # Nav2 variables
        self.nav_client = ActionClient(self, NavigateToPose, f"{self.robot_ns}/navigate_to_pose")
        self.current_goal_handle = None
        self.cancelling = False
        self.nav_state = 'IDLE'  # IDLE, NAV_TO_REDCAR, NAV_TO_PARTNER

        # ===============================
        # Subscribers
        # ===============================
        # 1) Self detection status
        self.create_subscription(Bool, 'object_detected', self.robot_detect_cb, 10)
        
        # 2) Self redcar target pose (calculated globally)
        self.create_subscription(PoseStamped, '/two_cops/target_state', self.target_state_cb, 10)
        
        # 3) Partner's detection status
        self.create_subscription(Bool, f"{self.partner_ns}/object_detected", self.partner_detect_cb, 10)
        
        # 4) Partner's current pose (AMCL)
        self.create_subscription(PoseWithCovarianceStamped, f"{self.partner_ns}/amcl_pose", self.partner_pose_cb, 10)

        # 10Hz Decision Loop (FSM)
        self.timer = self.create_timer(0.1, self.timer_cb)

    # ======================================================
    # Callbacks
    # ======================================================
    def robot_detect_cb(self, msg):
        self.robot_detected = msg.data

    def target_state_cb(self, msg):
        self.last_redcar_pose = msg

    def partner_detect_cb(self, msg):
        self.partner_detected = msg.data

    def partner_pose_cb(self, msg):
        self.last_partner_pose = msg.pose.pose

    # ======================================================
    # Nav2 Goal Management
    # ======================================================
    def send_nav_goal(self, pose, mode):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return
        
        self.nav_state = mode
        goal_msg = NavigateToPose.Goal()
        
        if isinstance(pose, PoseStamped):
            goal_msg.pose = pose
        else: # Pose type from amcl_pose
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose = pose
            goal_msg.pose = ps
        
        self.get_logger().info(f"[NAV] Sending goal: {mode}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def cancel_current_goal(self):
        if self.current_goal_handle is not None and not self.cancelling:
            self.get_logger().info("[NAV] Cancelling current goal for Tracking mode")
            self.cancelling = True
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future):
        self.cancelling = False
        self.current_goal_handle = None
        self.nav_state = 'IDLE'
        self.get_logger().info("[NAV] Cancellation completed")

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_state = 'IDLE'
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        self.current_goal_handle = None
        self.nav_state = 'IDLE'

    # ======================================================
    # FSM (Core Logic)
    # ======================================================
    def timer_cb(self):
        # 1) Robot is directly detecting the redcar -> Tracking Mode -> Cancel Nav2
        if self.robot_detected:
            if self.current_goal_handle is not None:
                self.cancel_current_goal()
            return

        # 2) If Nav2 is busy, wait for next decision loop
        if self.current_goal_handle is not None or self.cancelling:
            return

        # 3) Redcar coordinates available -> Highest priority goal
        if self.last_redcar_pose is not None:
            self.send_nav_goal(self.last_redcar_pose, mode="NAV_TO_REDCAR")
            return

        # 4) Redcar not found, but partner is currently detecting -> No specific target
        if self.partner_detected:
            return

        # 5) Partner not detecting, and partner pose is known -> Move towards partner
        if self.last_partner_pose is not None:
            self.send_nav_goal(self.last_partner_pose, mode="NAV_TO_PARTNER")


def main(args=None):
    rclpy.init(args=args)
    node = TransitionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
