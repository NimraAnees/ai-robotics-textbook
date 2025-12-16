#!/usr/bin/env python3
"""
ROS 2 Node Examples

This file contains various examples of ROS 2 nodes demonstrating different concepts
and best practices for the AI Robotics Textbook.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math


class MinimalPublisher(Node):
    """
    Example of a minimal publisher node
    """
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class MinimalSubscriber(Node):
    """
    Example of a minimal subscriber node
    """
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


class ParameterNode(Node):
    """
    Example of a node that uses parameters
    """
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('multiplier', 1.0)

        # Get parameter values
        self.my_param = self.get_parameter('my_parameter').value
        self.frequency = self.get_parameter('frequency').value
        self.multiplier = self.get_parameter('multiplier').value

        self.get_logger().info(f'My parameter is: {self.my_param}')
        self.get_logger().info(f'Frequency is: {self.frequency}')
        self.get_logger().info(f'Multiplier is: {self.multiplier}')

        # Use parameters to configure behavior
        self.publisher_ = self.create_publisher(String, 'parameter_output', 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Parameter value: {self.my_param}, Counter: {self.counter * self.multiplier}'
        self.publisher_.publish(msg)
        self.counter += 1


class TimerNode(Node):
    """
    Example of using timers in ROS 2
    """
    def __init__(self):
        super().__init__('timer_node')

        # Multiple timers with different rates
        self.slow_timer = self.create_timer(1.0, self.slow_timer_callback)
        self.fast_timer = self.create_timer(0.1, self.fast_timer_callback)

        self.publisher_slow = self.create_publisher(String, 'slow_topic', 10)
        self.publisher_fast = self.create_publisher(String, 'fast_topic', 10)

        self.slow_counter = 0
        self.fast_counter = 0

    def slow_timer_callback(self):
        msg = String()
        msg.data = f'Slow timer: {self.slow_counter}'
        self.publisher_slow.publish(msg)
        self.slow_counter += 1
        self.get_logger().info(f'Slow: {msg.data}')

    def fast_timer_callback(self):
        msg = String()
        msg.data = f'Fast timer: {self.fast_counter}'
        self.publisher_fast.publish(msg)
        self.fast_counter += 1


class ComplexNode(Node):
    """
    Example of a more complex node with multiple publishers, subscribers, and services
    """
    def __init__(self):
        super().__init__('complex_node')

        # Publishers
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscribers
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.input_callback,
            10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop

        # Internal state
        self.current_cmd = Twist()
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3']
        self.joint_states.position = [0.0, 0.0, 0.0]

        self.get_logger().info('Complex node initialized')

    def input_callback(self, msg):
        """Handle incoming commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Parse command and update internal state
        if 'forward' in msg.data.lower():
            self.current_cmd.linear.x = 0.5
        elif 'stop' in msg.data.lower():
            self.current_cmd.linear.x = 0.0

    def control_loop(self):
        """Main control loop running at 20Hz"""
        # Update joint positions (simulated movement)
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position[0] += 0.01  # Simple increment

        # Publish updated states
        self.joint_pub.publish(self.joint_states)
        self.velocity_pub.publish(self.current_cmd)


class LifecycleNode(Node):
    """
    Example of a node with proper resource management
    """
    def __init__(self):
        super().__init__('lifecycle_node')

        self.publisher_ = self.create_publisher(String, 'lifecycle_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize resources
        self.resource_handle = self.initialize_resource()
        self.get_logger().info('Lifecycle node initialized with resources')

    def initialize_resource(self):
        """Simulate resource initialization"""
        # In a real application, this might open files, connect to hardware, etc.
        return "resource_handle"

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.cleanup_resources()
        super().destroy_node()

    def cleanup_resources(self):
        """Clean up any resources used by the node"""
        self.get_logger().info(f'Cleaning up resource: {self.resource_handle}')
        # In a real application, this might close files, disconnect from hardware, etc.


def main(args=None):
    """
    Main function to demonstrate different node examples
    """
    rclpy.init(args=args)

    # Example 1: Minimal publisher
    print("Starting Minimal Publisher example...")
    minimal_publisher = MinimalPublisher()

    # Example 2: Minimal subscriber (to receive the publisher's messages)
    print("Starting Minimal Subscriber example...")
    minimal_subscriber = MinimalSubscriber()

    # Example 3: Parameter node
    print("Starting Parameter Node example...")
    param_node = ParameterNode()

    # Example 4: Timer node
    print("Starting Timer Node example...")
    timer_node = TimerNode()

    # Example 5: Complex node
    print("Starting Complex Node example...")
    complex_node = ComplexNode()

    # Example 6: Lifecycle node
    print("Starting Lifecycle Node example...")
    lifecycle_node = LifecycleNode()

    # Spin all nodes
    try:
        # Create an executor that can handle multiple nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(minimal_publisher)
        executor.add_node(minimal_subscriber)
        executor.add_node(param_node)
        executor.add_node(timer_node)
        executor.add_node(complex_node)
        executor.add_node(lifecycle_node)

        print("All nodes started. Press Ctrl+C to stop.")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down nodes...")
    finally:
        # Clean up all nodes
        minimal_publisher.destroy_node()
        minimal_subscriber.destroy_node()
        param_node.destroy_node()
        timer_node.destroy_node()
        complex_node.destroy_node()
        lifecycle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()