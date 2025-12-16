---
sidebar_label: 'Mini-Project: ROS 2 Control Stack'
sidebar_position: 3
---

# Mini-Project: ROS 2 Control Stack

This mini-project brings together all the concepts from Module 1 to build a complete ROS 2 control stack for a simulated humanoid robot. You'll implement a distributed system with multiple nodes that communicate to control the robot's movements.

## Project Overview

Your task is to create a ROS 2 control stack that allows you to command a simulated humanoid robot to move its joints to specific positions. The system will include:

- A joint state publisher node
- A trajectory controller node
- A command interface node
- A feedback monitoring node

## Learning Objectives

Upon completion of this project, you will be able to:
- Design and implement a complete ROS 2 control system
- Integrate multiple nodes with different responsibilities
- Use ROS 2 messages, services, and parameters effectively
- Test and validate your control system in simulation

## System Architecture

The control stack will consist of the following nodes:

### 1. Joint State Publisher Node
- Publishes current joint states using `sensor_msgs/JointState` messages
- Simulates joint positions, velocities, and efforts
- Publishes robot description via `/robot_description` parameter

### 2. Trajectory Controller Node
- Subscribes to trajectory commands via `trajectory_msgs/JointTrajectory` messages
- Implements simple joint interpolation
- Publishes feedback on execution status
- Uses services for configuration and control commands

### 3. Command Interface Node
- Provides a simple interface for sending trajectory commands
- Subscribes to user commands (keyboard, GUI, or service calls)
- Formats and sends trajectory messages to the controller

### 4. Feedback Monitor Node
- Subscribes to joint states and controller feedback
- Provides visualization of robot status
- Logs execution metrics and errors

## Implementation Steps

### Step 1: Set Up Package Structure

Create a ROS 2 package for your control stack:

```bash
mkdir -p ~/ros2_ws/src/humanoid_control_stack/src
cd ~/ros2_ws/src/humanoid_control_stack
```

Create the package.xml file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control_stack</name>
  <version>0.0.1</version>
  <description>ROS 2 Control Stack for Humanoid Robot</description>
  <maintainer email="student@university.edu">Student</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>trajectory_msgs</end>
  <depend>builtin_interfaces</depend>

  <exec_depend>python3-numpy</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 2: Implement Joint State Publisher

Create `humanoid_control_stack/humanoid_control_stack/joint_state_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import numpy as np
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        # Initialize joint positions (simulated humanoid)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'head_pan_joint', 'head_tilt_joint'
        ]

        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Joint State Publisher initialized')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        # Publish the message
        self.joint_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Implement Trajectory Controller

Create `humanoid_control_stack/humanoid_control_stack/trajectory_controller.py`:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.interpolate import interp1d

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        # Subscriber for trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Publisher for joint commands (in a real system, this would go to hardware interface)
        self.command_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Publisher for controller status
        self.status_pub = self.create_publisher(JointTrajectoryPoint, 'controller_status', 10)

        # Current joint states (initially from simulation)
        self.current_joint_states = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Controller state
        self.active_trajectory = None
        self.trajectory_start_time = None
        self.current_point_idx = 0

        # Timer for executing trajectories
        self.timer = self.create_timer(0.01, self.execute_trajectory)  # 100Hz

        self.get_logger().info('Trajectory Controller initialized')

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def trajectory_callback(self, msg):
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
        self.active_trajectory = msg
        self.trajectory_start_time = self.get_clock().now()
        self.current_point_idx = 0

    def execute_trajectory(self):
        if self.active_trajectory is None or self.current_joint_states is None:
            return

        # Calculate elapsed time since trajectory started
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.trajectory_start_time.nanoseconds) / 1e9

        # Check if trajectory is complete
        if self.current_point_idx >= len(self.active_trajectory.points):
            self.active_trajectory = None
            return

        # Get the target point based on time
        target_point = self.active_trajectory.points[self.current_point_idx]
        target_time = target_point.time_from_start.sec + target_point.time_from_start.nanosec / 1e9

        if elapsed >= target_time:
            # Move to next point
            self.current_point_idx += 1

            if self.current_point_idx < len(self.active_trajectory.points):
                # Publish the new target as a command
                cmd_msg = JointState()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.name = self.active_trajectory.joint_names
                cmd_msg.position = self.active_trajectory.points[self.current_point_idx-1].positions
                cmd_msg.velocity = self.active_trajectory.points[self.current_point_idx-1].velocities
                cmd_msg.effort = self.active_trajectory.points[self.current_point_idx-1].effort

                self.command_pub.publish(cmd_msg)

                # Publish status
                status_msg = self.active_trajectory.points[self.current_point_idx-1]
                self.status_pub.publish(status_msg)
            else:
                # Trajectory complete
                self.active_trajectory = None
                self.get_logger().info('Trajectory execution complete')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Implement Command Interface

Create `humanoid_control_stack/humanoid_control_stack/command_interface.py`:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')

        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)

        # Timer to send a demonstration trajectory
        self.timer = self.create_timer(5.0, self.send_demo_trajectory)

        self.demo_count = 0

        self.get_logger().info('Command Interface initialized')

    def send_demo_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'head_pan_joint', 'head_tilt_joint'
        ]

        # Create trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(msg.joint_names)  # Home position
        point1.velocities = [0.0] * len(msg.joint_names)
        point1.accelerations = [0.0] * len(msg.joint_names)
        point1.time_from_start = Duration(sec=1, nanosec=0)

        point2 = JointTrajectoryPoint()
        # Simple movement: lift left arm
        positions = [0.0] * len(msg.joint_names)
        positions[6] = 0.5  # left_shoulder_joint
        positions[7] = 0.3  # left_elbow_joint
        point2.positions = positions
        point2.velocities = [0.0] * len(msg.joint_names)
        point2.accelerations = [0.0] * len(msg.joint_names)
        point2.time_from_start = Duration(sec=2, nanosec=0)

        point3 = JointTrajectoryPoint()
        # Return to home position
        point3.positions = [0.0] * len(msg.joint_names)
        point3.velocities = [0.0] * len(msg.joint_names)
        point3.accelerations = [0.0] * len(msg.joint_names)
        point3.time_from_start = Duration(sec=3, nanosec=0)

        msg.points = [point1, point2, point3]

        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'Sent demo trajectory #{self.demo_count + 1}')
        self.demo_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Create Setup File

Create `humanoid_control_stack/setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_control_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@university.edu',
    description='ROS 2 Control Stack for Humanoid Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = humanoid_control_stack.joint_state_publisher:main',
            'trajectory_controller = humanoid_control_stack.trajectory_controller:main',
            'command_interface = humanoid_control_stack.command_interface:main',
        ],
    },
)
```

### Step 6: Create Launch File

Create `humanoid_control_stack/launch/control_stack_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control_stack',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='humanoid_control_stack',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen'
        ),
        Node(
            package='humanoid_control_stack',
            executable='command_interface',
            name='command_interface',
            output='screen'
        )
    ])
```

## Testing Your Implementation

1. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select humanoid_control_stack
   source install/setup.bash
   ```

2. **Run the system**:
   ```bash
   ros2 launch humanoid_control_stack control_stack_launch.py
   ```

3. **Monitor the system**:
   ```bash
   # Check published joint states
   ros2 topic echo /joint_states

   # Check controller status
   ros2 topic echo /controller_status
   ```

## Project Deliverables

Submit the following for evaluation:

1. Complete source code for all nodes
2. Launch file to start the entire system
3. A README.md file explaining your implementation
4. Screenshots or logs showing successful execution of the demonstration trajectory
5. A brief report (1-2 pages) describing:
   - Design decisions made during implementation
   - Challenges encountered and how you addressed them
   - How your system follows ROS 2 best practices

## Evaluation Criteria

Your project will be evaluated on:

- **Functionality** (40%): Does the system correctly implement the required nodes and communication patterns?
- **Code Quality** (25%): Is the code well-structured, documented, and following Python/ROS 2 best practices?
- **Design** (20%): Does the architecture effectively separate concerns and follow ROS 2 principles?
- **Testing** (15%): Does the implementation include proper error handling and validation?

## Extension Challenges

For additional learning, consider implementing:

1. **PID Controllers**: Add PID controllers for more precise joint control
2. **Safety Features**: Implement joint limits and collision avoidance
3. **Advanced Trajectories**: Support for smooth, interpolated trajectories
4. **Real-time Performance**: Optimize for real-time constraints

## Summary

This mini-project demonstrates the practical application of ROS 2 concepts in creating a functional control system for a humanoid robot. You've implemented a distributed system with multiple nodes that communicate effectively to achieve a common goal.

## Next Steps

After completing this mini-project, you'll have a solid foundation in ROS 2 concepts and implementation. In Module 2, you'll integrate this control stack with simulation environments to test your robot in virtual worlds.