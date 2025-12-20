#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class TrackingNode(Node):
    def __init__(self):
        super().__init__('move_robot_override')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle = None
        self.distance = None
        self.has_seen = False
        self.last_detect_time = None
        self.no_detect_limit = 1.5
        self.object_detected = False
        self.create_subscription(Float32, 'detected_angle', self.angle_cb, 10)
        self.create_subscription(Float32, 'detected_distance', self.dist_cb, 10)
        self.create_subscription(Bool, 'object_detected', self.detect_cb, 10)
        self.target_dist = 1.0
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)
        self.get_logger().info("TrackingNode Override 10Hz started - cmd_vel output as needed") #

    # ---------------- Callbacks ----------------
    def detect_cb(self, msg):
        self.object_detected = msg.data
        if msg.data:
            if not self.has_seen:
                self.get_logger().info("First detection - starting tracking") #
            self.has_seen = True
            self.last_detect_time = time.time()

    def angle_cb(self, msg):
        self.angle = msg.data

    def dist_cb(self, msg):
        self.distance = msg.data

    # ---------------- Main Control ----------------
    def timer_callback(self):
        twist = Twist()
        # 0) Before first detection -> do not publish any messages
        if not self.has_seen:
            return
        # 1) If object not detected -> do not publish
        if not self.object_detected:
            return
        # 2) If angle/distance values are missing -> do not publish
        if self.angle is None or self.distance is None:
            return
        
        angle = self.angle
        dist = self.distance
        
        # 3) If within target distance -> stop publishing (no stop packet)
        if dist <= self.target_dist:
            self.get_logger().info("Target distance reached - stopping publication") #
            return
            
        # Only publish when actual movement is required
        # Rotation
        deadzone = 2.0
        twist.angular.z = -angle * 0.01 if abs(angle) > deadzone else 0.0
        
        # Forward Movement
        err = dist - self.target_dist
        twist.linear.x = min(0.9, err * 0.4)
        
        self.pub.publish(twist)
        self.get_logger().info(
            f"[MOVE] angle={angle:.1f}°, dist={dist:.2f}m -> v={twist.linear.x:.2f}, w={twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
