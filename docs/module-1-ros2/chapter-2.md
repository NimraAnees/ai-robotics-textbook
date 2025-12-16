---
sidebar_label: 'Chapter 2: Python Integration with rclpy'
sidebar_position: 2
---

# Chapter 2: Python Integration with rclpy

This chapter focuses on implementing ROS 2 concepts using Python and the `rclpy` library. You'll learn how to create nodes, publish and subscribe to topics, implement services, and work with actions in Python.

## Learning Objectives

After completing this chapter, you will be able to:
- Create ROS 2 nodes using Python and rclpy
- Implement publishers and subscribers for message passing
- Create and use services for request-response communication
- Work with actions for long-running operations
- Structure Python code following ROS 2 best practices

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides Python APIs that are conceptually similar to the underlying ROS client library implementations (rcl). Using `rclpy`, you can create ROS 2 nodes, publish and subscribe to topics, provide and use services, and create and use actions.

## Setting Up Your Environment

Before writing ROS 2 Python code, ensure your environment is properly configured:

```bash
source /opt/ros/humble/setup.bash
```

## Creating Your First ROS 2 Node

Let's start by creating a simple ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

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

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Subscribers

To create a subscriber that receives messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

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

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services

Creating a service server:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Creating a service client:

```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Actions

Creating an action server:

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result
```

## Parameter Handling

ROS 2 nodes can accept parameters that can be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')

        # Get parameter value
        my_param = self.get_parameter('my_parameter').value
        self.get_logger().info(f'My parameter is: {my_param}')
```

## Launch Files

Launch files allow you to start multiple nodes at once with specific configurations:

```xml
<launch>
  <node pkg="my_package" exec="minimal_publisher" name="publisher" output="screen">
    <param name="my_parameter" value="custom_value"/>
  </node>
  <node pkg="my_package" exec="minimal_subscriber" name="subscriber" output="screen"/>
</launch>
```

## Best Practices

1. **Node Structure**: Organize your node as a class that inherits from `rclpy.node.Node`
2. **Resource Management**: Always call `destroy_node()` when shutting down
3. **Logging**: Use `self.get_logger().info()` for debugging and status messages
4. **Error Handling**: Implement proper error handling for network and communication issues
5. **Parameter Validation**: Validate parameters at startup to ensure configuration is correct

## Code Organization

Structure your ROS 2 Python packages following these conventions:

```
my_robot_package/
├── setup.py
├── package.xml
├── resource/
│   └── my_robot_package
├── my_robot_package/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── publisher_node.py
│   │   └── subscriber_node.py
│   └── utils/
│       ├── __init__.py
│       └── helper_functions.py
└── launch/
    └── my_launch_file.launch.py
```

## Summary

This chapter provided a comprehensive introduction to implementing ROS 2 concepts in Python using rclpy. You've learned how to create nodes, implement different communication patterns, handle parameters, and structure your code following best practices.

## Exercises

1. **Implementation Exercise**: Create a ROS 2 node that publishes sensor data (simulated IMU readings) and another node that subscribes to this data and logs it.
2. **Service Exercise**: Implement a service that calculates the distance between two points in 2D space, with the node that provides the service and a client that calls it.
3. **Architecture Exercise**: Design and implement a simple robot controller with separate nodes for sensor processing, path planning, and motor control, all communicating through ROS 2 topics.

## Next Steps

In the next section, we'll explore URDF (Unified Robot Description Format) for modeling humanoid robots, building on the communication foundation established in this chapter.