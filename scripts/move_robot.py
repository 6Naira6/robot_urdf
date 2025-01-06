#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0

        self.get_logger().info("MoveRobotNode has started.")

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.get_logger().info(f"Current Position -> x: {self.position_x}, y: {self.position_y}")

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Publishing -> linear_x: {self.linear_velocity}, angular_z: {self.angular_velocity}")

    def set_velocity(self, linear_velocity, angular_velocity):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()

    try:
        while rclpy.ok():
            linear_velocity = float(input("Enter linear velocity (m/s): "))
            angular_velocity = float(input("Enter angular velocity (rad/s): "))
            node.set_velocity(linear_velocity, angular_velocity)

            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveRobotNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

