# ROS 2 Launch System Diagram

## Description
This diagram shows the structure and workflow of the ROS 2 launch system.

## Architecture Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 Launch System                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────┐│
│  │  Launch File    │────▶│  Launch File    │────▶│  Launch     ││
│  │  (main.launch.py)│    │  (robot.launch.py)│    │  Service    ││
│  └─────────────────┘     └─────────────────┘     └─────────────┘│
│         │                        │                       │      │
│         ▼                        ▼                       ▼      │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────┐│
│  │ Launch Manager  │◀────│ Launch Manager  │◀────│  ros2 launch││
│  │ (Composable)    │     │ (Standalone)    │     │  command    ││
│  └─────────────────┘     └─────────────────┘     └─────────────┘│
│         │                        │                       │      │
│         ▼                        ▼                       ▼      │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                   Running Nodes                             ││
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   ││
│  │  │Node 1    │  │Node 2    │  │Node 3    │  │Node 4    │   ││
│  │  │(Controller│  │(Sensor)  │  │(Publisher│  │(Subscriber│   ││
│  │  │Manager)  │  │          │  │)         │  │)         │   ││
│  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘   ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## Key Components:
- **Launch Files**: Python scripts that define which nodes to launch and their configurations
- **Launch Manager**: Manages the lifecycle of nodes, handles startup, shutdown, and error recovery
- **Launch Service**: Backend service that executes launch files
- **Composable Nodes**: Nodes that can run within the same process for efficiency

## Parameters and Configuration:
```
Launch File Structure:
├── Node Definitions
├── Parameters
├── Remappings
├── Conditions
└── Event Handlers
```

## Benefits:
- Centralized node management
- Parameter configuration at launch time
- Conditional node startup
- Automatic node restart on failure