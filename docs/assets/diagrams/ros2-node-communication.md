# ROS 2 Node Communication Diagram

## Description
This diagram illustrates how nodes communicate in ROS 2 using topics, services, and actions.

## Architecture Overview
```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Publisher     │     │                 │     │   Subscriber    │
│   Node A        │────▶│   Topic         │────▶│   Node B        │
│  (e.g. Sensor)  │     │   (e.g. /cmd_vel) │     │ (e.g. Controller) │
└─────────────────┘     └─────────────────┘     └─────────────────┘

┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Client        │     │   Service       │     │   Server        │
│   Node C        │────▶│   (e.g. /add_two_ints) │   Node D        │
│  (e.g. Requestor) │    │                 │     │ (e.g. Calculator) │
└─────────────────┘     └─────────────────┘     └─────────────────┘

┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Action Client │     │   Action Server │     │   Action Server │
│   Node E        │────▶│   (e.g. MoveRobot) │◀───│   Node F        │
│  (e.g. Planner)  │     │                 │     │ (e.g. Robot)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

## Key Concepts:
- **Nodes**: Independent processes that communicate with each other
- **Topics**: Asynchronous, many-to-many communication using publish/subscribe pattern
- **Services**: Synchronous, one-to-one communication using request/response pattern
- **Actions**: Asynchronous, goal-oriented communication with feedback and status

## DDS Implementation:
- Data Distribution Service (DDS) handles message passing between nodes
- Supports different Quality of Service (QoS) policies
- Enables communication across different machines and networks