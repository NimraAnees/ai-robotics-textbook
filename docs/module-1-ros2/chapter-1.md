---
sidebar_label: 'Chapter 1: ROS 2 Architecture'
sidebar_position: 1
---

# Chapter 1: ROS 2 Architecture

This chapter introduces the fundamental architecture of ROS 2 (Robot Operating System 2), the communication backbone that enables distributed robotic systems to work together seamlessly.

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the core concepts of ROS 2 architecture
- Identify and describe the main components (nodes, topics, services, actions)
- Understand the communication patterns in ROS 2
- Design basic communication structures for robotic systems

## Introduction to ROS 2

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Unlike traditional monolithic approaches, ROS 2 uses a distributed architecture where different components (nodes) communicate with each other through a publish-subscribe model.

ROS 2 represents a significant evolution from ROS 1, addressing key concerns like security, real-time performance, and support for commercial applications. It's designed to be suitable for real applications, from research and prototyping to production systems.

## Core Architecture Components

### Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically performs a specific task and communicates with other nodes to achieve more complex robot behaviors.

:::info Key Concept
Think of nodes as individual processes or services that each handle a specific function, like perception, control, or planning.
:::

### Topics and Messages

**Topics** are named buses over which nodes exchange messages. The communication is based on a publish-subscribe pattern where publishers send messages to a topic and subscribers receive messages from a topic. This enables loose coupling between nodes.

**Messages** are the data structures that are passed between nodes. They are defined using the `.msg` file format and contain primitive data types like integers, floats, booleans, and strings, as well as arrays of these types.

### Services

**Services** provide a request-response communication pattern. Unlike topics which are asynchronous, services are synchronous - the client sends a request and waits for a response from the server. Services are useful for operations that need a direct response, like requesting specific robot actions or configuration changes.

### Actions

**Actions** are a more advanced communication pattern that combines features of both topics and services. They're designed for long-running tasks that require feedback during execution and the ability to cancel the operation. Actions are perfect for navigation goals, manipulation tasks, or any operation that takes time to complete.

## Communication Patterns

### Publisher-Subscriber Pattern

The publisher-subscriber pattern enables loose coupling between nodes. Publishers send data to topics without knowing who (if anyone) is subscribed. Subscribers receive data from topics without knowing who published the data.

```python
# Example publisher
publisher = node.create_publisher(StringMsg, 'topic_name', 10)
msg = StringMsg()
msg.data = 'Hello World'
publisher.publish(msg)
```

### Service Client-Server Pattern

The service pattern provides direct request-response communication between nodes.

```python
# Example service server
service = node.create_service(AddTwoInts, 'add_two_ints', callback_function)

# Example service client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

### Action Client-Server Pattern

Actions provide feedback during long-running operations and the ability to cancel.

```python
# Example action server
action_server = ActionServer(node, Fibonacci, 'fibonacci', execute_callback)

# Example action client
action_client = ActionClient(node, Fibonacci, 'fibonacci')
```

## Practical Example: Simple Robot System

Let's consider a simple robot system with three nodes:

1. **Sensor Node**: Publishes sensor data (e.g., laser scans) to a topic
2. **Controller Node**: Subscribes to sensor data, processes it, and publishes motor commands
3. **Motor Node**: Subscribes to motor commands and controls the physical motors

This architecture allows each component to be developed, tested, and maintained independently while working together as a system.

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings that allow you to fine-tune communication behavior. QoS settings control aspects like reliability, durability, and history of messages, which is crucial for real-time and safety-critical applications.

- **Reliability**: Whether messages are guaranteed to be delivered
- **Durability**: Whether late-joining subscribers get old messages
- **History**: How many messages to store for late-joining subscribers

## Summary

ROS 2's distributed architecture enables the creation of complex robotic systems by breaking them down into smaller, manageable components that communicate through well-defined interfaces. Understanding these core concepts is essential for designing effective robotic systems.

## Exercises

1. **Conceptual Exercise**: Draw a diagram showing the communication between nodes in a mobile robot with laser scanner, camera, and motor controllers.
2. **Research Exercise**: Investigate how QoS settings affect communication in a ROS 2 system and when you might use different configurations.
3. **Design Exercise**: Design a ROS 2 architecture for a humanoid robot with multiple sensors (IMU, cameras, joint encoders) and actuators (arm joints, leg joints, head pan-tilt).

## Next Steps

In the next chapter, we'll explore how to implement these concepts using Python and the `rclpy` library, diving into practical ROS 2 programming.