---
sidebar_label: 'Chapter 3: Launch Files and Parameters'
sidebar_position: 3
---

# Chapter 3: Launch Files and Parameters

This chapter covers ROS 2 launch files and parameter management, essential tools for configuring and starting complex robotic systems with multiple nodes.

## Learning Objectives

After completing this chapter, you will be able to:
- Create and use launch files to start multiple nodes simultaneously
- Configure node parameters through launch files
- Manage complex robot configurations using parameter files
- Use command-line arguments with launch files

## Introduction to Launch Files

Launch files are a crucial component of ROS 2 that allow you to start multiple nodes with a single command. They provide a way to define complex robot configurations and ensure that all necessary components start with the correct parameters and in the correct order.

Launch files replace the launch files from ROS 1 but use a different implementation based on Python rather than XML. This provides more flexibility and the ability to use conditional logic, loops, and other programming constructs.

## Creating Launch Files

### Basic Launch File Structure

A launch file is a Python file that defines a `generate_launch_description()` function:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42},
            ]
        )
    ])
```

### Launch File with Parameters

Launch files can accept arguments that can be passed from the command line:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,

        Node(
            package='my_package',
            executable='controller_node',
            name=[LaunchConfiguration('robot_name'), '_controller'],
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')},
            ]
        )
    ])
```

## Parameter Management

### Parameter Files

Parameters can be stored in YAML files for easy management and sharing:

```yaml
# my_robot_params.yaml
/**:
  ros__parameters:
    use_sim_time: false
    update_rate: 50.0
    publish_tf: true

controller_node:
  ros__parameters:
    kp: 1.0
    ki: 0.1
    kd: 0.05

sensor_node:
  ros__parameters:
    frame_id: 'base_link'
    range_min: 0.1
    range_max: 10.0
```

### Loading Parameters in Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_robot_package')

    # Path to parameter file
    params_file = os.path.join(pkg_share, 'config', 'my_robot_params.yaml')

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='controller_node',
            parameters=[params_file]
        ),
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='sensor_node',
            parameters=[params_file]
        )
    ])
```

## Advanced Launch Concepts

### Conditional Launch

Launch files can include conditional logic:

```python
from launch import LaunchDescription, LaunchCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start camera node'
    )

    camera_node = Node(
        package='camera_package',
        executable='camera_node',
        name='camera_node',
        condition=LaunchCondition(
            expression=[LaunchConfiguration('use_camera')]
        )
    )

    return LaunchDescription([
        use_camera,
        camera_node
    ])
```

### Including Other Launch Files

You can include other launch files to build complex systems:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include another launch file
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('other_package'),
                'launch',
                'other_launch.py'
            )
        )
    )

    return LaunchDescription([
        other_launch,
        # Additional nodes...
    ])
```

## Practical Example: Humanoid Robot Launch

Here's a practical example of a launch file for a humanoid robot:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Get package directories
    pkg_share = get_package_share_directory('humanoid_robot_bringup')
    robot_description_path = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': open(robot_description_path).read()}
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'humanoid_robot.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,
        use_rviz,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        rviz
    ])
```

## Best Practices

### Organize Launch Files
- Group related nodes in separate launch files
- Use descriptive names for launch files
- Keep launch files focused on specific functionality

### Parameter Management
- Use YAML files for complex parameter sets
- Group related parameters logically
- Use meaningful parameter names
- Document parameter purposes

### Error Handling
- Validate parameters at startup
- Provide meaningful error messages
- Use appropriate default values

## Launch Commands

To run a launch file:

```bash
# Basic launch
ros2 launch my_package my_launch.py

# With arguments
ros2 launch my_package my_launch.py use_sim_time:=true robot_name:=my_robot

# With parameters from file
ros2 launch my_package my_launch.py --params-file path/to/params.yaml
```

## Summary

Launch files and parameters are essential for managing complex robotic systems. They allow you to start multiple nodes with proper configurations and ensure that your robot system is set up correctly for different operating conditions.

## Exercises

1. **Launch File Exercise**: Create a launch file that starts a joint state publisher, robot state publisher, and a simple controller node for a basic robot.
2. **Parameter Exercise**: Create a YAML parameter file for a mobile robot with differential drive, including parameters for wheel separation, wheel radius, and maximum velocities.
3. **Conditional Launch Exercise**: Modify your launch file to conditionally start a visualization node based on a launch argument.

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format) for modeling humanoid robots, which will be used extensively with the launch files and parameters you've learned about in this chapter.