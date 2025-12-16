#!/usr/bin/env python3
"""
ROS 2 Topic Examples

This file contains various examples of ROS 2 topic usage demonstrating
publish-subscribe patterns, message types, and best practices.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Bool, Header
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, Vector3, Point
from builtin_interfaces.msg import Time
import time
import math
import random


class TopicPublisherExample(Node):
    """
    Example publisher demonstrating various message types and patterns
    """
    def __init__(self):
        super().__init__('topic_publisher_example')

        # Different publishers for different message types
        self.string_pub = self.create_publisher(String, 'string_topic', 10)
        self.int_pub = self.create_publisher(Int32, 'int_topic', 10)
        self.float_pub = self.create_publisher(Float32, 'float_topic', 10)
        self.bool_pub = self.create_publisher(Bool, 'bool_topic', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to publish messages at different rates
        self.string_timer = self.create_timer(1.0, self.publish_string)
        self.int_timer = self.create_timer(0.5, self.publish_int)
        self.float_timer = self.create_timer(0.2, self.publish_float)
        self.complex_timer = self.create_timer(0.1, self.publish_complex_messages)

        self.string_counter = 0
        self.int_counter = 0

    def publish_string(self):
        """Publish string messages"""
        msg = String()
        msg.data = f'Hello from publisher: {self.string_counter}'
        self.string_pub.publish(msg)
        self.get_logger().info(f'Published string: {msg.data}')
        self.string_counter += 1

    def publish_int(self):
        """Publish integer messages"""
        msg = Int32()
        msg.data = self.int_counter
        self.int_pub.publish(msg)
        self.get_logger().info(f'Published int: {msg.data}')
        self.int_counter += 1

    def publish_float(self):
        """Publish float messages"""
        msg = Float32()
        msg.data = random.uniform(-10.0, 10.0)
        self.float_pub.publish(msg)
        self.get_logger().info(f'Published float: {msg.data:.2f}')

    def publish_complex_messages(self):
        """Publish more complex message types"""
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [math.sin(time.time()), math.cos(time.time()), 0.5 * math.sin(2 * time.time())]
        joint_msg.velocity = [0.0] * 3
        joint_msg.effort = [0.0] * 3
        self.joint_state_pub.publish(joint_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        # Set orientation (simplified - just using sin/cos for demonstration)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(time.time() * 0.5)
        imu_msg.orientation.w = math.cos(time.time() * 0.5)
        # Set angular velocity
        imu_msg.angular_velocity.z = 0.5 * math.cos(time.time() * 0.5)
        # Set linear acceleration
        imu_msg.linear_acceleration.x = 9.81 * math.sin(time.time() * 2.0)
        self.imu_pub.publish(imu_msg)

        # Publish velocity commands
        vel_msg = Twist()
        vel_msg.linear.x = 0.5 + 0.2 * math.sin(time.time() * 0.3)
        vel_msg.angular.z = 0.3 * math.sin(time.time() * 0.7)
        self.cmd_vel_pub.publish(vel_msg)


class TopicSubscriberExample(Node):
    """
    Example subscriber demonstrating various message types and patterns
    """
    def __init__(self):
        super().__init__('topic_subscriber_example')

        # Different subscribers for different message types
        self.string_sub = self.create_subscription(String, 'string_topic', self.string_callback, 10)
        self.int_sub = self.create_subscription(Int32, 'int_topic', self.int_callback, 10)
        self.float_sub = self.create_subscription(Float32, 'float_topic', self.float_callback, 10)
        self.bool_sub = self.create_subscription(Bool, 'bool_topic', self.bool_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Store last received values
        self.last_string = None
        self.last_int = None
        self.last_float = None
        self.last_joint_positions = []

    def string_callback(self, msg):
        """Handle string messages"""
        self.last_string = msg.data
        self.get_logger().info(f'Received string: {msg.data}')

    def int_callback(self, msg):
        """Handle integer messages"""
        self.last_int = msg.data
        self.get_logger().info(f'Received int: {msg.data}')

    def float_callback(self, msg):
        """Handle float messages"""
        self.last_float = msg.data
        self.get_logger().info(f'Received float: {msg.data:.2f}')

    def bool_callback(self, msg):
        """Handle boolean messages"""
        self.get_logger().info(f'Received bool: {msg.data}')

    def joint_callback(self, msg):
        """Handle joint state messages"""
        self.last_joint_positions = list(msg.position)
        self.get_logger().info(f'Received joint states: {msg.position}')

    def imu_callback(self, msg):
        """Handle IMU messages"""
        self.get_logger().info(f'Received IMU - Orientation: ({msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f})')

    def cmd_vel_callback(self, msg):
        """Handle velocity command messages"""
        self.get_logger().info(f'Received cmd_vel - Linear: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), Angular: ({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})')


class TopicFilterExample(Node):
    """
    Example of filtering and processing messages
    """
    def __init__(self):
        super().__init__('topic_filter_example')

        # Subscribe to raw sensor data
        self.raw_sub = self.create_subscription(Float32, 'raw_sensor_data', self.raw_callback, 10)

        # Publish processed data
        self.processed_pub = self.create_publisher(Float32, 'processed_sensor_data', 10)

        # For filtering
        self.raw_data_history = []
        self.filter_size = 5

    def raw_callback(self, msg):
        """Process raw sensor data with filtering"""
        # Add to history
        self.raw_data_history.append(msg.data)

        # Keep only recent values
        if len(self.raw_data_history) > self.filter_size:
            self.raw_data_history.pop(0)

        # Calculate average (simple filter)
        if len(self.raw_data_history) > 0:
            filtered_value = sum(self.raw_data_history) / len(self.raw_data_history)

            # Publish filtered data
            filtered_msg = Float32()
            filtered_msg.data = filtered_value
            self.processed_pub.publish(filtered_msg)

            self.get_logger().info(f'Raw: {msg.data:.2f}, Filtered: {filtered_value:.2f}')


class TopicSynchronizerExample(Node):
    """
    Example of synchronizing multiple topics (simplified version)
    """
    def __init__(self):
        super().__init__('topic_synchronizer_example')

        # Subscribe to multiple related topics
        self.imu_sub = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # Publish combined state
        self.state_pub = self.create_publisher(String, 'robot_state', 10)

        # Store latest values with timestamps
        self.last_imu = None
        self.last_joints = None
        self.last_imu_time = None
        self.last_joints_time = None

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.last_imu = msg
        self.last_imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.process_combined_state()

    def joint_callback(self, msg):
        """Handle joint state data"""
        self.last_joints = msg
        self.last_joints_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.process_combined_state()

    def process_combined_state(self):
        """Process combined state when both messages are available"""
        if self.last_imu and self.last_joints:
            # Simple check if messages are reasonably synchronized (within 0.1 seconds)
            if abs(self.last_imu_time - self.last_joints_time) < 0.1:
                # Create combined state message
                state_msg = String()
                state_msg.data = f'IMU Orientation: ({self.last_imu.orientation.z:.2f}), Joint 1: {self.last_joints.position[0]:.2f}'
                self.state_pub.publish(state_msg)
                self.get_logger().info(f'Combined state: {state_msg.data}')


class TopicQoSExample(Node):
    """
    Example of using different QoS settings
    """
    def __init__(self):
        super().__init__('topic_qos_example')

        # Publisher with different QoS settings
        # Using default QoS (reliable, volatile)
        self.default_pub = self.create_publisher(String, 'default_qos_topic', 10)

        # Publisher with transient local durability (for latching)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.transient_pub = self.create_publisher(String, 'transient_qos_topic', transient_qos)

        # Timer to publish messages
        self.timer = self.create_timer(2.0, self.publish_messages)

    def publish_messages(self):
        """Publish messages with different QoS"""
        msg = String()
        msg.data = f'QoS message at {time.time():.2f}'

        self.default_pub.publish(msg)
        self.transient_pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    """
    Main function to demonstrate topic examples
    """
    rclpy.init(args=args)

    # Create nodes
    publisher_node = TopicPublisherExample()
    subscriber_node = TopicSubscriberExample()
    filter_node = TopicFilterExample()
    sync_node = TopicSynchronizerExample()
    qos_node = TopicQoSExample()

    print("Starting topic examples...")
    print("Publisher node will publish various message types")
    print("Subscriber node will receive and log messages")
    print("Filter node will demonstrate data processing")
    print("Synchronizer node will combine multiple topics")
    print("QoS node will demonstrate Quality of Service settings")

    try:
        # Create executor and add all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)
        executor.add_node(filter_node)
        executor.add_node(sync_node)
        executor.add_node(qos_node)

        print("All topic example nodes started. Press Ctrl+C to stop.")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down topic example nodes...")
    finally:
        # Clean up
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        filter_node.destroy_node()
        sync_node.destroy_node()
        qos_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()