---
sidebar_label: 'Chapter 5: Sensor-Actuator-Control Pipelines'
sidebar_position: 5
---

# Chapter 5: Sensor-Actuator-Control Pipelines

This chapter explores the integration of sensors, actuators, and control systems in ROS 2, forming the complete feedback loops necessary for autonomous robot operation. You'll learn how to create sensor-actuator-control pipelines that enable robots to perceive their environment, make decisions, and execute actions.

## Learning Objectives

After completing this chapter, you will be able to:
- Design sensor-actuator feedback loops for robotic systems
- Implement control pipelines that process sensor data and command actuators
- Integrate various sensor types (IMU, encoders, cameras, LiDAR) into control systems
- Implement safety mechanisms and error handling in control pipelines

## Introduction to Sensor-Actuator-Controller Architecture

The sensor-actuator-control pipeline forms the foundation of autonomous robotic systems. It creates a feedback loop where sensors provide information about the robot's state and environment, controllers process this information to make decisions, and actuators execute the resulting commands.

The typical pipeline follows this pattern:
```
Sensors → Sensor Processing → State Estimation → Controller → Actuator Commands → Actuators → Robot → Environment → Sensors (feedback)
```

## Sensor Types and Integration

### Joint Position Sensors (Encoders)

Joint position sensors provide feedback about the current position of robot joints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateProcessor(Node):
    def __init__(self):
        super().__init__('joint_state_processor')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for processed joint data
        self.joint_pub = self.create_publisher(JointState, 'processed_joint_states', 10)

        # Store previous joint states for velocity calculation
        self.prev_positions = {}
        self.prev_times = {}

        self.get_logger().info('Joint State Processor initialized')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9

        processed_msg = JointState()
        processed_msg.header = msg.header
        processed_msg.name = msg.name
        processed_msg.position = msg.position
        processed_msg.velocity = []
        processed_msg.effort = msg.effort

        # Calculate velocities from position changes
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            if name in self.prev_positions:
                dt = current_time - self.prev_times[name]
                if dt > 0:
                    vel = (pos - self.prev_positions[name]) / dt
                    processed_msg.velocity.append(vel)
                else:
                    processed_msg.velocity.append(0.0)
            else:
                processed_msg.velocity.append(0.0)

            # Update stored values
            self.prev_positions[name] = pos
            self.prev_times[name] = current_time

        # Publish processed joint states
        self.joint_pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    processor = JointStateProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### IMU (Inertial Measurement Unit) Sensors

IMU sensors provide orientation, angular velocity, and linear acceleration:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Publisher for processed orientation
        self.orientation_pub = self.create_publisher(Vector3, 'robot_orientation', 10)

        # Store previous data for filtering
        self.prev_orientation = None
        self.orientation_history = []

        self.get_logger().info('IMU Processor initialized')

    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Convert to Euler angles (roll, pitch, yaw)
        rotation = R.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=True)

        # Create Vector3 message with Euler angles
        orientation_msg = Vector3()
        orientation_msg.x = euler[0]  # roll
        orientation_msg.y = euler[1]  # pitch
        orientation_msg.z = euler[2]  # yaw

        # Publish processed orientation
        self.orientation_pub.publish(orientation_msg)

        # Store for filtering if needed
        self.orientation_history.append(euler)
        if len(self.orientation_history) > 10:
            self.orientation_history.pop(0)

def main(args=None):
    rclpy.init(args=args)
    processor = ImuProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Camera and Vision Sensors

Camera sensors provide visual information for perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Create OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed image
        self.processed_pub = self.create_publisher(Image, 'camera/image_processed', 10)

        self.get_logger().info('Vision Processor initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            processed_msg.header = msg.header

            # Publish processed image
            self.processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    processor = VisionProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Control System Implementation

### PID Controller Example

A PID (Proportional-Integral-Derivative) controller is fundamental for precise control:

```python
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, setpoint, current_value, dt=None):
        current_time = time.time()

        if dt is None and self.prev_time is not None:
            dt = current_time - self.prev_time
        elif dt is None:
            dt = 0.01  # Default time step

        # Calculate error
        error = setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp output to limits
        output = max(self.min_output, min(self.max_output, output))

        # Store values for next iteration
        self.prev_error = error
        self.prev_time = current_time

        return output
```

### Joint Controller Node

Here's a complete joint controller that integrates sensor feedback with actuator commands:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import numpy as np

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Controller parameters
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        # PID controllers for each joint
        self.pid_controllers = {}
        for joint_name in self.joint_names:
            self.pid_controllers[joint_name] = PIDController(
                kp=2.0, ki=0.1, kd=0.05,
                min_output=-10.0, max_output=10.0
            )

        # Current joint states
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.current_velocities = {name: 0.0 for name in self.joint_names}

        # Target positions (initially current positions)
        self.target_positions = self.current_positions.copy()

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        self.get_logger().info('Joint Controller initialized')

    def joint_state_callback(self, msg):
        # Update current joint states
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.current_positions:
                self.current_positions[name] = pos
                self.current_velocities[name] = vel

    def control_loop(self):
        # Calculate control commands for each joint
        commands = []

        for joint_name in self.joint_names:
            current_pos = self.current_positions[joint_name]
            target_pos = self.target_positions[joint_name]

            # Compute control effort using PID
            effort = self.pid_controllers[joint_name].compute(
                target_pos, current_pos
            )

            commands.append(effort)

        # Publish commands
        command_msg = Float64MultiArray()
        command_msg.data = commands
        self.command_pub.publish(command_msg)

    def set_target_positions(self, positions_dict):
        """Set target positions for joints"""
        for joint_name, target_pos in positions_dict.items():
            if joint_name in self.target_positions:
                self.target_positions[joint_name] = target_pos

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    # Example: Set a target position after startup
    def set_initial_target():
        controller.set_target_positions({
            'left_hip_joint': 0.1,
            'right_hip_joint': 0.1
        })

    # Schedule initial target after a short delay
    timer = controller.create_timer(1.0, set_initial_target)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Error Handling

### Safety Monitor Node

A safety monitor watches the system and implements safety mechanisms:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool
import numpy as np

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Safety parameters
        self.joint_limits = {
            'left_hip_joint': (-1.5, 1.5),
            'left_knee_joint': (0.0, 2.5),
            'left_ankle_joint': (-0.5, 0.5),
            'right_hip_joint': (-1.5, 1.5),
            'right_knee_joint': (0.0, 2.5),
            'right_ankle_joint': (-0.5, 0.5)
        }

        self.max_velocity = 5.0  # rad/s
        self.max_tilt = 0.5  # rad (about 28 degrees)

        # Safety state
        self.emergency_stop = False
        self.safety_violation = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Publisher for safety state
        self.safety_pub = self.create_publisher(Bool, 'safety_status', 10)

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz

        self.get_logger().info('Safety Monitor initialized')

    def joint_state_callback(self, msg):
        if self.emergency_stop:
            return

        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_limits:
                # Check position limits
                min_limit, max_limit = self.joint_limits[name]
                if pos < min_limit or pos > max_limit:
                    self.safety_violation = True
                    self.get_logger().error(f'Joint {name} exceeded position limits: {pos}')
                    self.trigger_emergency_stop()
                    return

                # Check velocity limits
                if abs(vel) > self.max_velocity:
                    self.safety_violation = True
                    self.get_logger().error(f'Joint {name} exceeded velocity limits: {vel}')
                    self.trigger_emergency_stop()
                    return

    def imu_callback(self, msg):
        if self.emergency_stop:
            return

        # Extract orientation from IMU
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Convert to Euler angles
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_quat(quat)
        euler = rotation.as_euler('xyz')

        # Check tilt angles
        roll, pitch, yaw = euler
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.safety_violation = True
            self.get_logger().error(f'Robot exceeded tilt limits: roll={roll:.2f}, pitch={pitch:.2f}')
            self.trigger_emergency_stop()
            return

    def safety_check(self):
        # Publish current safety status
        safety_msg = Bool()
        safety_msg.data = not self.emergency_stop
        self.safety_pub.publish(safety_msg)

    def trigger_emergency_stop(self):
        self.emergency_stop = True
        self.get_logger().error('EMERGENCY STOP TRIGGERED - Safety violation detected!')

def main(args=None):
    rclpy.init(args=args)
    monitor = SafetyMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Pipeline Integration

Here's an example of how to integrate all components into a complete sensor-actuator-control pipeline:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

class CompleteControlPipeline(Node):
    def __init__(self):
        super().__init__('complete_control_pipeline')

        # Control state
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.current_velocities = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}

        # PID controllers
        self.pid_controllers = {}
        for name in self.joint_names:
            self.pid_controllers[name] = PIDController(
                kp=2.0, ki=0.1, kd=0.05,
                min_output=-10.0, max_output=10.0
            )

        # Safety state
        self.safety_enabled = True
        self.emergency_stop = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.safety_sub = self.create_subscription(
            Bool,
            'safety_status',
            self.safety_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Complete Control Pipeline initialized')

    def joint_state_callback(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.current_positions:
                self.current_positions[name] = pos
                self.current_velocities[name] = vel

    def imu_callback(self, msg):
        # Process IMU data if needed for control
        pass

    def safety_callback(self, msg):
        self.safety_enabled = msg.data

    def control_loop(self):
        if not self.safety_enabled or self.emergency_stop:
            # Send zero commands during safety stop
            zero_commands = [0.0] * len(self.joint_names)
            command_msg = Float64MultiArray()
            command_msg.data = zero_commands
            self.command_pub.publish(command_msg)
            return

        # Calculate control commands
        commands = []
        for name in self.joint_names:
            current_pos = self.current_positions[name]
            target_pos = self.target_positions[name]

            effort = self.pid_controllers[name].compute(target_pos, current_pos)
            commands.append(effort)

        # Publish commands
        command_msg = Float64MultiArray()
        command_msg.data = commands
        self.command_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    pipeline = CompleteControlPipeline()
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for Complete Pipeline

Create `launch/sensor_actuator_pipeline_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Joint state publisher (simulated robot)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),

        # Sensor processors
        Node(
            package='my_robot_control',
            executable='joint_state_processor',
            name='joint_state_processor',
            output='screen'
        ),

        Node(
            package='my_robot_control',
            executable='imu_processor',
            name='imu_processor',
            output='screen'
        ),

        Node(
            package='my_robot_control',
            executable='vision_processor',
            name='vision_processor',
            output='screen'
        ),

        # Controllers
        Node(
            package='my_robot_control',
            executable='joint_controller',
            name='joint_controller',
            output='screen'
        ),

        # Safety monitor
        Node(
            package='my_robot_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen'
        ),

        # Complete pipeline
        Node(
            package='my_robot_control',
            executable='complete_control_pipeline',
            name='complete_control_pipeline',
            output='screen'
        )
    ])
```

## Best Practices

### Real-Time Considerations
- Use appropriate control frequencies (typically 100Hz for joint control)
- Minimize computational complexity in control loops
- Use efficient data structures and algorithms
- Consider thread safety for concurrent access

### Safety First
- Always implement emergency stop functionality
- Set appropriate limits for positions, velocities, and efforts
- Monitor for safety violations continuously
- Design fail-safe behaviors

### Modularity
- Separate sensor processing, control, and actuation logic
- Use ROS 2's composability features when appropriate
- Design nodes to be reusable across different robots
- Follow ROS 2 design patterns and conventions

## Summary

Sensor-actuator-control pipelines form the backbone of autonomous robotic systems. By properly integrating sensors, controllers, and actuators with appropriate safety mechanisms, you can create robust systems that operate reliably in real-world conditions.

## Exercises

1. **Pipeline Exercise**: Create a complete sensor-actuator-control pipeline for a simple mobile robot with wheel encoders and motor controllers.
2. **Safety Exercise**: Implement additional safety checks for a humanoid robot, including collision detection and joint torque limits.
3. **Integration Exercise**: Design a launch file that starts all components of your sensor-actuator-control pipeline and test it with a simulated robot.

## Next Steps

With the foundational ROS 2 concepts, launch files, URDF modeling, and sensor-actuator pipelines covered, you're ready to work on the Module 1 mini-project: building a complete ROS 2 control stack. This will integrate all the concepts learned in this module into a functional system.