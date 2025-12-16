---
sidebar_label: 'Exercises'
sidebar_position: 6
---

# ROS 2 Exercises

This collection of exercises reinforces the concepts learned in Module 1: Robotic Nervous System (ROS 2). Each exercise builds upon the previous concepts and provides hands-on practice with ROS 2 development.

## Exercise 1: Basic Node Communication

### Objective
Create a simple publisher-subscriber system to understand ROS 2 communication patterns.

### Instructions
1. Create a ROS 2 package called `basic_communication`
2. Implement a publisher node that publishes "Hello, Robot!" messages to a topic called `greetings` every 2 seconds
3. Implement a subscriber node that listens to the `greetings` topic and logs the received messages
4. Create a launch file that starts both nodes simultaneously
5. Test the system by running the launch file and verifying message exchange

### Code Template
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        # TODO: Create publisher
        # TODO: Create timer to publish messages
        self.count = 0

    def timer_callback(self):
        # TODO: Create and publish message
        pass

def main(args=None):
    # TODO: Initialize and run node
    pass
```

### Expected Outcome
- Publisher sends messages every 2 seconds
- Subscriber receives and logs messages
- System runs without errors
- Proper use of ROS 2 node structure

### Learning Goals
- Understanding node structure in ROS 2
- Implementing publishers and subscribers
- Using timers for periodic tasks
- Creating and using launch files

## Exercise 2: Service Implementation

### Objective
Create a service that performs calculations to understand request-response communication.

### Instructions
1. Create a service that calculates the distance between two 2D points
2. The service should accept x1, y1, x2, y2 coordinates and return the Euclidean distance
3. Create a service server node
4. Create a service client node that calls the service with test coordinates
5. Test with multiple coordinate pairs to verify functionality

### Code Template
```python
# distance_calculator.py
from example_interfaces.srv import AddTwoInts  # You'll need to create a custom service
import rclpy
from rclpy.node import Node
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        # TODO: Create service server
        pass

    def calculate_distance(self, request, response):
        # TODO: Calculate distance and set response
        pass

def main(args=None):
    # TODO: Initialize and run service server
    pass
```

### Expected Outcome
- Service correctly calculates distances
- Client successfully calls service and receives responses
- Error handling for invalid inputs
- Proper service interface design

### Learning Goals
- Understanding service communication pattern
- Creating custom service interfaces
- Implementing service servers and clients
- Error handling in ROS 2 services

## Exercise 3: Action Implementation

### Objective
Create an action that simulates a robot moving to a goal to understand long-running operations.

### Instructions
1. Create an action that simulates a robot moving to a specified position
2. The action should provide feedback on progress (percentage complete)
3. Implement cancellation functionality
4. Create an action server node
5. Create an action client node that sends goals and monitors progress
6. Test with multiple goals and cancellation scenarios

### Code Template
```python
# move_robot_action.py
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
import rclpy
from rclpy.node import Node

class MoveRobotAction(Node):
    def __init__(self):
        super().__init__('move_robot_action')
        # TODO: Create action server
        pass

    def execute_callback(self, goal_handle):
        # TODO: Implement action execution with feedback
        pass

def main(args=None):
    # TODO: Initialize and run action server
    pass
```

### Expected Outcome
- Action server provides feedback during execution
- Client receives progress updates
- Cancellation works properly
- Proper state management throughout action lifecycle

### Learning Goals
- Understanding action communication pattern
- Implementing action servers and clients
- Providing feedback during long operations
- Handling goal cancellation

## Exercise 4: Parameter Management

### Objective
Create a node that uses parameters to configure its behavior.

### Instructions
1. Create a node that changes its behavior based on parameters
2. Use parameters for publishing rate, message content, and operational mode
3. Create a parameter file with different configurations
4. Create a launch file that loads parameters from the file
5. Test different parameter configurations

### Expected Outcome
- Node behavior changes based on parameters
- Parameters can be loaded from YAML files
- Launch file properly configures the node
- Parameters are validated at startup

### Learning Goals
- Parameter declaration and usage
- YAML parameter files
- Launch file parameter loading
- Parameter validation

## Exercise 5: URDF Robot Model

### Objective
Create a URDF model for a simple mobile robot with differential drive.

### Instructions
1. Create a URDF file for a robot with a base, two wheels, and a caster
2. Include visual and collision properties for each link
3. Define joints connecting the wheels to the base
4. Add transmissions for the wheels
5. Test the URDF with `check_urdf` and visualize in RViz

### URDF Template
```xml
<?xml version="1.0"?>
<robot name="differential_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <!-- TODO: Define base visual properties -->
    </visual>
    <collision>
      <!-- TODO: Define base collision properties -->
    </collision>
    <inertial>
      <!-- TODO: Define base inertial properties -->
    </inertial>
  </link>

  <!-- Wheels -->
  <!-- TODO: Define left and right wheel links and joints -->
</robot>
```

### Expected Outcome
- URDF validates without errors
- Robot model displays correctly in RViz
- Kinematic structure is correct
- Proper visual and collision properties

### Learning Goals
- URDF structure and components
- Link and joint definitions
- Visual and collision properties
- Robot kinematics

## Exercise 6: Sensor Integration

### Objective
Create a node that processes sensor data from multiple sources.

### Instructions
1. Create a sensor fusion node that subscribes to IMU and wheel encoder data
2. Combine sensor data to estimate robot pose
3. Publish the fused estimate to a new topic
4. Add basic error handling for sensor failures
5. Test with simulated sensor data

### Expected Outcome
- Node successfully subscribes to multiple sensor topics
- Sensor data is properly fused
- Fused data is published correctly
- Error handling works for missing sensor data

### Learning Goals
- Multi-topic subscription
- Sensor data processing
- Message synchronization
- Error handling in sensor systems

## Exercise 7: Control System

### Objective
Implement a simple PID controller for robot joint control.

### Instructions
1. Create a PID controller node for a single joint
2. Subscribe to current joint position and desired position
3. Calculate control effort using PID algorithm
4. Publish control commands to the joint
5. Test with simulated joint dynamics

### Code Template
```python
class JointPIDController(Node):
    def __init__(self):
        super().__init__('joint_pid_controller')
        # TODO: Create subscribers and publishers
        # TODO: Initialize PID controller parameters
        pass

    def position_callback(self, msg):
        # TODO: Calculate control effort
        # TODO: Publish control command
        pass
```

### Expected Outcome
- PID controller stabilizes the joint at desired positions
- Control parameters can be tuned for performance
- System handles disturbances appropriately
- Proper control loop timing

### Learning Goals
- PID control implementation
- Control system architecture
- Real-time control considerations
- Disturbance rejection

## Exercise 8: Safety System

### Objective
Create a safety monitor that watches robot state and implements emergency stops.

### Instructions
1. Create a safety node that monitors joint positions and velocities
2. Define safety limits for each joint
3. Implement emergency stop functionality when limits are exceeded
4. Publish safety status to coordinate with other nodes
5. Test safety system with out-of-bounds values

### Expected Outcome
- Safety system detects violations promptly
- Emergency stop prevents dangerous conditions
- Safety status is properly communicated
- System recovers safely after violations

### Learning Goals
- Safety system design
- Real-time monitoring
- Emergency response systems
- Safety-critical system architecture

## Exercise 9: Complete System Integration

### Objective
Combine multiple concepts into a complete robot system.

### Instructions
1. Integrate sensor processing, control, and safety systems
2. Use launch files to start all components
3. Implement parameter management for the complete system
4. Test system behavior with various scenarios
5. Document the system architecture

### Expected Outcome
- All components work together seamlessly
- System starts correctly with launch files
- Parameters are properly configured
- Safety system protects the complete system
- Architecture is well-documented

### Learning Goals
- System integration
- Component coordination
- Architecture design
- Documentation practices

## Exercise 10: Performance Optimization

### Objective
Optimize a ROS 2 system for better performance.

### Instructions
1. Profile a simple ROS 2 system to identify bottlenecks
2. Optimize message passing and processing
3. Tune QoS settings for better performance
4. Implement efficient data structures
5. Measure performance improvements

### Expected Outcome
- System performance is measurably improved
- Proper QoS settings are used
- Efficient data processing is implemented
- Performance metrics are documented

### Learning Goals
- Performance profiling
- QoS optimization
- Efficient programming practices
- System performance measurement

## Submission Requirements

For each exercise, submit:
1. Complete source code
2. Launch files (if applicable)
3. Parameter files (if applicable)
4. Brief report explaining your implementation
5. Screenshots or logs showing successful execution
6. Any challenges encountered and how you resolved them

## Evaluation Criteria

Exercises will be evaluated based on:
- **Correctness** (40%): Does the implementation work as specified?
- **Code Quality** (25%): Is the code well-structured, documented, and following ROS 2 best practices?
- **Understanding** (20%): Does the implementation show understanding of ROS 2 concepts?
- **Documentation** (15%): Is the code and approach well-documented?

## Extension Challenges

For advanced students, consider these extensions:
1. Add multi-robot coordination to exercises
2. Implement more complex sensor fusion algorithms
3. Add machine learning components to control systems
4. Create a web interface for robot monitoring
5. Implement advanced safety mechanisms like STO (Safe Torque Off)

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- API Documentation: https://docs.ros.org/en/humble/p/rclpy/
- Community: https://answers.ros.org/

These exercises provide comprehensive hands-on experience with ROS 2 concepts, building from basic to advanced topics and preparing students for real-world robot development.