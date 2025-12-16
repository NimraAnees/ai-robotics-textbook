#!/usr/bin/env python3
"""
ROS 2 Service Examples

This file contains various examples of ROS 2 service usage demonstrating
request-response patterns, custom services, and best practices.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import AddTwoInts, SetBool, Trigger
from std_msgs.msg import String
import time
import math
import threading
from functools import partial


class ServiceServerExample(Node):
    """
    Example service server demonstrating various service types
    """
    def __init__(self):
        super().__init__('service_server_example')

        # Create services with different callback groups for concurrency
        self.add_two_ints_srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.set_bool_srv = self.create_service(
            SetBool,
            'set_bool',
            self.set_bool_callback
        )

        self.trigger_srv = self.create_service(
            Trigger,
            'trigger_service',
            self.trigger_callback
        )

        # Additional service with custom logic
        self.math_op_srv = self.create_service(
            AddTwoInts,  # Reusing existing service type for example
            'math_operation',
            self.math_operation_callback
        )

        # State for set_bool service
        self.bool_state = False

        self.get_logger().info('Service server initialized')

    def add_two_ints_callback(self, request, response):
        """Callback for adding two integers"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Adding {request.a} + {request.b} = {response.sum}')
        return response

    def set_bool_callback(self, request, response):
        """Callback for setting a boolean state"""
        self.bool_state = request.data
        response.success = True
        response.message = f'Bool state set to {self.bool_state}'
        self.get_logger().info(f'Set bool to {request.data}')
        return response

    def trigger_callback(self, request, response):
        """Callback for trigger service"""
        response.success = True
        response.message = f'Triggered at {time.time():.2f}'
        self.get_logger().info('Trigger service called')
        return response

    def math_operation_callback(self, request, response):
        """Callback for more complex math operations"""
        # Simulate some processing time
        time.sleep(0.1)

        # Perform different operations based on values
        if request.a > request.b:
            result = request.a * request.b
        elif request.a < request.b:
            result = request.b - request.a
        else:
            result = request.a + request.b

        response.sum = result
        self.get_logger().info(f'Math op: {request.a}, {request.b} -> {result}')
        return response


class ServiceClientExample(Node):
    """
    Example service client demonstrating various service calls
    """
    def __init__(self):
        super().__init__('service_client_example')

        # Create clients for the services
        self.add_two_ints_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.set_bool_client = self.create_client(SetBool, 'set_bool')
        self.trigger_client = self.create_client(Trigger, 'trigger_service')
        self.math_op_client = self.create_client(AddTwoInts, 'math_operation')

        # Wait for services to be available
        while not self.add_two_ints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add two ints service not available, waiting again...')

        while not self.set_bool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set bool service not available, waiting again...')

        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trigger service not available, waiting again...')

        while not self.math_op_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Math operation service not available, waiting again...')

        # Timer to call services periodically
        self.timer = self.create_timer(3.0, self.call_services)
        self.call_count = 0

        self.get_logger().info('Service client initialized')

    def call_services(self):
        """Call different services periodically"""
        self.get_logger().info(f'Calling services - round {self.call_count}')

        # Call add_two_ints service
        self.call_add_two_ints(10, 5)

        # Call set_bool service
        self.call_set_bool(self.call_count % 2 == 0)

        # Call trigger service
        self.call_trigger()

        # Call math operation service
        self.call_math_operation(self.call_count, self.call_count + 1)

        self.call_count += 1

    def call_add_two_ints(self, a, b):
        """Call the add_two_ints service"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Use async call to avoid blocking
        future = self.add_two_ints_client.call_async(request)
        future.add_done_callback(partial(self.add_two_ints_callback, a=a, b=b))

    def add_two_ints_callback(self, future, a, b):
        """Callback for add_two_ints response"""
        try:
            response = future.result()
            self.get_logger().info(f'Result of {a} + {b} = {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def call_set_bool(self, value):
        """Call the set_bool service"""
        request = SetBool.Request()
        request.data = value

        future = self.set_bool_client.call_async(request)
        future.add_done_callback(partial(self.set_bool_callback, value=value))

    def set_bool_callback(self, future, value):
        """Callback for set_bool response"""
        try:
            response = future.result()
            self.get_logger().info(f'Set bool to {value}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Set bool service call failed: {e}')

    def call_trigger(self):
        """Call the trigger service"""
        request = Trigger.Request()

        future = self.trigger_client.call_async(request)
        future.add_done_callback(self.trigger_callback)

    def trigger_callback(self, future):
        """Callback for trigger response"""
        try:
            response = future.result()
            self.get_logger().info(f'Trigger response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Trigger service call failed: {e}')

    def call_math_operation(self, a, b):
        """Call the math operation service"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.math_op_client.call_async(request)
        future.add_done_callback(partial(self.math_operation_callback, a=a, b=b))

    def math_operation_callback(self, future, a, b):
        """Callback for math operation response"""
        try:
            response = future.result()
            self.get_logger().info(f'Math op {a}, {b} result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Math operation service call failed: {e}')


class AsyncServiceClientExample(Node):
    """
    Example of using async service calls with threading
    """
    def __init__(self):
        super().__init__('async_service_client_example')

        # Create clients
        self.add_two_ints_client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service
        while not self.add_two_ints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add two ints service not available, waiting again...')

        # Timer to make concurrent service calls
        self.timer = self.create_timer(2.0, self.make_concurrent_calls)
        self.call_counter = 0

    def make_concurrent_calls(self):
        """Make multiple concurrent service calls"""
        self.get_logger().info(f'Making concurrent calls - batch {self.call_counter}')

        # Make several concurrent calls
        for i in range(3):
            a = self.call_counter * 10 + i
            b = self.call_counter * 5 + i + 1
            self.call_add_two_ints_async(a, b)

        self.call_counter += 1

    def call_add_two_ints_async(self, a, b):
        """Make an async service call"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.add_two_ints_client.call_async(request)
        future.add_done_callback(partial(self.async_callback, a=a, b=b))

    def async_callback(self, future, a, b):
        """Callback for async service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Async result: {a} + {b} = {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Async service call failed: {e}')


class ServiceWithTimeoutExample(Node):
    """
    Example of service calls with timeout handling
    """
    def __init__(self):
        super().__init__('service_with_timeout_example')

        # Create client
        self.add_two_ints_client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service with timeout
        timeout_count = 0
        while not self.add_two_ints_client.wait_for_service(timeout_sec=1.0):
            timeout_count += 1
            self.get_logger().info('Add two ints service not available, waiting again...')
            if timeout_count > 10:  # Give up after 10 seconds
                self.get_logger().error('Service not available after 10 seconds')
                return

        # Timer to make service calls with timeout
        self.timer = self.create_timer(5.0, self.call_with_timeout)
        self.timeout_call_count = 0

    def call_with_timeout(self):
        """Make service call with timeout handling"""
        self.get_logger().info(f'Making timeout call - attempt {self.timeout_call_count}')

        request = AddTwoInts.Request()
        request.a = self.timeout_call_count
        request.b = self.timeout_call_count * 2

        # Make async call and wait with timeout
        future = self.add_two_ints_client.call_async(request)

        # We'll use a separate thread to handle the timeout
        # In a real application, you might use a timer to cancel the future
        future.add_done_callback(self.timeout_callback)

        self.timeout_call_count += 1

    def timeout_callback(self, future):
        """Callback for timeout service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Timeout call result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Timeout call failed: {e}')


class ServiceComposedNode(Node):
    """
    Example of a node that provides services and also uses other services
    """
    def __init__(self):
        super().__init__('service_composed_node')

        # Provide a service
        self.complex_calc_srv = self.create_service(
            AddTwoInts,
            'complex_calculation',
            self.complex_calculation_callback
        )

        # Use other services
        self.add_two_ints_client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service we'll use
        while not self.add_two_ints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Remote add service not available, waiting...')

        # Timer to periodically call external services
        self.timer = self.create_timer(4.0, self.periodic_service_call)

    def complex_calculation_callback(self, request, response):
        """Complex calculation that might use other services"""
        self.get_logger().info(f'Complex calculation requested: {request.a}, {request.b}')

        # Simulate complex calculation that uses another service
        # First, do the basic addition using the external service
        temp_request = AddTwoInts.Request()
        temp_request.a = request.a
        temp_request.b = request.b

        # For this example, we'll just do the calculation locally
        # In a real scenario, you might need to call other services
        # and wait for their responses
        result = request.a + request.b

        # Then do additional processing
        final_result = result * 2  # Example of additional processing

        response.sum = final_result
        self.get_logger().info(f'Complex calculation result: {response.sum}')
        return response

    def periodic_service_call(self):
        """Periodically call external services"""
        request = AddTwoInts.Request()
        request.a = int(time.time()) % 100
        request.b = int(time.time() * 10) % 50

        future = self.add_two_ints_client.call_async(request)
        future.add_done_callback(self.periodic_callback)

    def periodic_callback(self, future):
        """Callback for periodic service call"""
        try:
            response = future.result()
            self.get_logger().info(f'Periodic call result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Periodic call failed: {e}')


def main(args=None):
    """
    Main function to demonstrate service examples
    """
    rclpy.init(args=args)

    # Create all nodes
    server_node = ServiceServerExample()
    client_node = ServiceClientExample()
    async_client_node = AsyncServiceClientExample()
    timeout_node = ServiceWithTimeoutExample()
    composed_node = ServiceComposedNode()

    print("Starting service examples...")
    print("Server node provides various services")
    print("Client node calls services periodically")
    print("Async client makes concurrent service calls")
    print("Timeout example demonstrates timeout handling")
    print("Composed node both provides and uses services")

    try:
        # Use multi-threaded executor to handle multiple nodes and async calls
        executor = MultiThreadedExecutor()
        executor.add_node(server_node)
        executor.add_node(client_node)
        executor.add_node(async_client_node)
        executor.add_node(timeout_node)
        executor.add_node(composed_node)

        print("All service example nodes started. Press Ctrl+C to stop.")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down service example nodes...")
    finally:
        # Clean up all nodes
        server_node.destroy_node()
        client_node.destroy_node()
        async_client_node.destroy_node()
        timeout_node.destroy_node()
        composed_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()