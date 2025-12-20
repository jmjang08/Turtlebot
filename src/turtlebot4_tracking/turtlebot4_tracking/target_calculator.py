#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class TargetCalculator(Node):
    def __init__(self):
        super().__init__('target_calculator')

        # ===== Internal State Variables =====
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None       # [rad]
        self.last_distance = None    # [m]
        self.last_angle_deg = None   # [deg]
        self.object_detected = False

        # ===== Subscribers =====
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )
        self.create_subscription(
            Float32,
            'detected_distance',
            self.distance_callback,
            10
        )
        self.create_subscription(
            Float32,
            'detected_angle',
            self.angle_callback,
            10
        )
        self.create_subscription(
            Bool,
            'object_detected',
            self.detected_callback,
            10
        )

        # ===== Publishers =====
        self.target_state = self.create_publisher(
            PoseStamped,
            '/two_cops/target_state',
            10
        )
        
        self.get_logger().info("TargetCalculator initialized")

    # ===========================
    # Callbacks
    # ===========================
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Save the robot's position and orientation in the map frame"""
        pose = msg.pose.pose
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        q = pose.orientation
        self.robot_yaw = self.quat_to_yaw(q)

    def distance_callback(self, msg: Float32):
        """Distance to the red car from YOLODepthFusion (m)"""
        self.last_distance = msg.data

    def angle_callback(self, msg: Float32):
        """Angle to the red car from YOLODepthFusion (deg)"""
        self.last_angle_deg = msg.data

    def detected_callback(self, msg: Bool):
        """
        Red car detection status.
        If True, calculate the global coordinates based on robot pose + distance/angle and publish.
        """
        self.object_detected = msg.data
        if not self.object_detected:
            return

        # 1. Data Validity Check
        if (self.robot_x is None or
            self.robot_y is None or
            self.robot_yaw is None or
            self.last_distance is None or
            self.last_angle_deg is None):
            self.get_logger().warn(
                "object_detected=True but not enough data (pose/distance/angle)"
            )
            return

        # --------------------------------------------------
        # 2. Distance Limit Filter (Clamping: > 1m to 1m)
        # --------------------------------------------------
        distance_used = min(self.last_distance, 1.0)
        if self.last_distance > 1.0:
            self.get_logger().info(
                f"[INFO] Raw distance {self.last_distance:.2f} > 1.0m -> Using clamped 1.0m"
            )

        # --------------------------------------------------
        # 3. Compute Global Coordinates
        # --------------------------------------------------
        target_x, target_y = self.compute_global_target(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            distance_used,             # Using clamped distance
            self.last_angle_deg
        )

        # --------------------------------------------------
        # 4. Convert to PoseStamped and Publish
        # --------------------------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = 0.0
        
        # Maintain robot's yaw direction for the target pose
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        
        self.target_state.publish(pose_msg)
        
        self.get_logger().info(
            f"[RED CAR] global pose -> "
            f"x={target_x:.3f}, y={target_y:.3f}, "
            f"dist(used)={distance_used:.2f}m (raw={self.last_distance:.2f}), "
            f"angle={self.last_angle_deg:.1f}deg"
        )

    # ===========================
    # Utility Functions
    # ===========================
    @staticmethod
    def quat_to_yaw(q) -> float:
        """
        Converts geometry_msgs/Quaternion to yaw (rad)
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    @staticmethod
    def compute_global_target(robot_x, robot_y, robot_yaw, distance, angle_deg):
        """
        Calculates the red car's global coordinates using robot's global pose and relative distance/angle.
        angle_deg: Relative horizontal angle from robot's forward direction (deg)
        """
        angle_rad = math.radians(angle_deg)
        target_angle = robot_yaw + angle_rad
        target_x = robot_x + distance * math.cos(target_angle)
        target_y = robot_y + distance * math.sin(target_angle)
        return target_x, target_y


def main(args=None):
    rclpy.init(args=args)
    node = TargetCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
