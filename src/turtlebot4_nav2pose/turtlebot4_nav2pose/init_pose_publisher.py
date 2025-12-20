# my_nav_utils/initial_pose_publisher.py

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'initialpose')
        self.declare_parameter('publish_delay', 3.0)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.publish_delay = float(self.get_parameter('publish_delay').value)

        # Initial pose publisher
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.topic,
            10
        )

        # Publish once after a specified delay
        self.timer = self.create_timer(self.publish_delay, self._publish_once)

        self.get_logger().info(
            f'InitialPosePublisher initialized: topic={self.topic}, '
            f'frame_id={self.frame_id}, x={self.x}, y={self.y}, yaw={self.yaw}'
        )

    def _publish_once(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Set Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # Set standard AMCL-level covariance (x, y, yaw)
        cov = [0.0] * 36
        cov[0] = 0.25   # Variance for x
        cov[7] = 0.25   # Variance for y
        cov[35] = 0.0685  # Variance for yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f'Initial pose published: ({self.x:.3f}, {self.y:.3f}, yaw={self.yaw:.3f} rad)'
        )

        # Stop timer (publish only once)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
