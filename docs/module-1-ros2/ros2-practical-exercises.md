# ROS 2 Practical Exercises

## Exercise 1: Basic Publisher/Subscriber

### Objective
Create a simple publisher that sends "Hello, ROS 2!" messages and a subscriber that receives and prints them.

### Instructions
1. Create a new ROS 2 package called `hello_ros2_demo`
2. Implement a publisher node that publishes string messages to the topic `/hello_msg`
3. Implement a subscriber node that subscribes to `/hello_msg` and prints received messages
4. Create a launch file to start both nodes simultaneously
5. Test the communication between nodes

### Expected Outcome
- Publisher sends messages every 2 seconds
- Subscriber prints received messages to the console
- Communication works without errors

### Solution Outline
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher = self.create_publisher(String, 'hello_msg', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    hello_publisher = HelloPublisher()
    rclpy.spin(hello_publisher)
    hello_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Service Server/Client

### Objective
Create a service that adds two integers and a client that calls this service.

### Instructions
1. Define a custom service interface for adding two integers
2. Implement a service server that performs the addition
3. Implement a service client that sends requests to the server
4. Test the service communication

### Expected Outcome
- Service server responds to requests with the sum of two numbers
- Client receives and prints the results

## Exercise 3: Action Server/Client

### Objective
Create an action that simulates a robot moving to a goal position.

### Instructions
1. Define a custom action interface for robot movement
2. Implement an action server that simulates movement to a goal
3. Implement an action client that sends goals to the server
4. The server should provide feedback during execution
5. Test the action communication

### Expected Outcome
- Action client sends goals to the server
- Server provides feedback during execution
- Server sends result when movement is complete

## Exercise 4: Parameter Server

### Objective
Use ROS 2 parameters to configure node behavior at runtime.

### Instructions
1. Create a node that uses parameters for configuration
2. Set parameters from the command line when launching the node
3. Use the parameter service to change parameters at runtime
4. Implement parameter validation callbacks

### Expected Outcome
- Node behavior changes based on parameter values
- Parameters can be changed during runtime
- Parameter validation works correctly

## Exercise 5: Launch System

### Objective
Create a complex launch file that starts multiple nodes with parameters.

### Instructions
1. Create multiple nodes (publisher, subscriber, service server)
2. Create a launch file that starts all nodes
3. Use launch arguments to configure behavior
4. Use conditions to start nodes based on parameters
5. Add event handlers for error handling

### Expected Outcome
- All nodes start correctly from the launch file
- Launch arguments work as expected
- Conditions properly control node startup

## Exercise 6: URDF Robot Model

### Objective
Create a URDF model for a simple humanoid robot.

### Instructions
1. Create a URDF file describing a simple humanoid robot with at least 6 joints
2. Include visual and collision properties for each link
3. Add joint limits and safety controllers
4. Visualize the robot in RViz
5. Generate a launch file to display the robot

### Expected Outcome
- URDF file loads without errors
- Robot displays correctly in RViz
- All joints move properly

## Exercise 7: ROS 2 with Real Hardware

### Objective
Connect ROS 2 to a physical robot or sensor.

### Instructions
1. Choose a hardware platform (e.g., Raspberry Pi robot, Arduino with ROS bridge)
2. Install appropriate ROS 2 packages for the hardware
3. Create nodes that interface with the hardware
4. Test communication between ROS 2 and hardware
5. Implement error handling for hardware failures

### Expected Outcome
- ROS 2 nodes successfully communicate with hardware
- Data flows correctly between ROS 2 and hardware
- Error handling works properly

## Exercise 8: Multi-Robot Communication

### Objective
Set up communication between multiple ROS 2 nodes across different machines.

### Instructions
1. Set up two machines to communicate over the same network
2. Configure ROS 2 for multi-machine communication
3. Create publisher on one machine and subscriber on another
4. Test communication between machines
5. Configure DDS settings for optimal performance

### Expected Outcome
- Nodes on different machines can communicate
- Message passing works across network
- Network configuration is properly set up

## Evaluation Criteria

For each exercise, students will be evaluated on:
- Correct implementation of ROS 2 concepts
- Proper use of ROS 2 tools and conventions
- Code quality and documentation
- Successful completion of the exercise objectives
- Understanding of underlying principles demonstrated through debugging and problem-solving