# Unity Scenes for Robotics Simulation

This directory contains Unity scene files for robotics visualization and simulation. These scenes work with the Unity Robotics Package to provide advanced visualization capabilities for humanoid robots.

## Setup Requirements

### Unity Version
- Unity 2021.3 LTS or later recommended
- Unity Robotics Package (com.unity.robotics.urp)
- Unity Simulation Package (optional, for advanced features)

### Installation
1. Install Unity Hub and create a new 3D project
2. Add the Unity Robotics Package via Package Manager
3. Import the necessary robotics assets and components

## Scene Structure

### Basic Humanoid Scene
- `basic_humanoid_scene.unity` - Simple scene with humanoid robot and basic environment
- Contains basic lighting and camera setup
- Includes basic terrain and obstacles

### Advanced Simulation Scene
- `advanced_simulation_scene.unity` - Complex scene with detailed physics and sensors
- Includes multiple sensor configurations
- Advanced lighting and rendering settings

## ROS 2 Integration

### ROS-TCP-Connector
The scenes use the ROS-TCP-Connector for communication with ROS 2:
- Publisher/Subscriber communication
- Service calls
- Action servers/clients

### Coordinate System
- Unity uses left-handed coordinate system (X-right, Y-up, Z-forward)
- Automatic conversion to ROS right-handed system (X-forward, Y-left, Z-up)

## Scene Components

### Robot Prefabs
- Humanoid robot models with articulated joints
- Pre-configured sensors (LiDAR, cameras, IMU)
- Physics colliders and rigidbodies

### Environment Objects
- Terrain with realistic materials
- Interactive objects and obstacles
- Lighting and atmospheric effects

### Sensor Components
- 3D LiDAR simulation using raycasting
- RGB and depth camera simulation
- IMU simulation with noise models

## Example Scene Configuration

Here's a basic example of how to set up a Unity scene for robotics:

```
Main Camera
├── Robot (Prefab)
│   ├── Base Link
│   ├── Joint1
│   ├── Joint2
│   ├── Sensors
│   │   ├── LiDAR Sensor
│   │   ├── Camera Sensor
│   │   └── IMU Sensor
│   └── Physics Components
├── Environment
│   ├── Ground Plane
│   ├── Obstacles
│   └── Lighting
└── ROS Connection Manager
    ├── TCP Connector
    └── Message Handlers
```

## Physics Configuration

Unity physics settings for robotics simulation:
- Fixed Timestep: 0.02 (50 Hz for stable physics)
- Maximum Allowed Timestep: 0.3333333
- Solver Iterations: 10-20
- Solver Velocity Iterations: 2-4

## Performance Optimization

For real-time robotics simulation:
- Use Level of Detail (LOD) for complex models
- Optimize mesh complexity
- Use occlusion culling for large environments
- Limit physics substeps for real-time performance

## Integration with ROS 2

### Message Types Supported
- sensor_msgs/LaserScan
- sensor_msgs/Image
- sensor_msgs/Imu
- geometry_msgs/Twist
- std_msgs/Float64MultiArray

### Example Connection Script
```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>("lidar_scan");
    }
}
```

## Scene Creation Workflow

1. Create a new Unity 3D project
2. Import the Unity Robotics Package
3. Create the robot model with proper joint configuration
4. Add sensor components to the robot
5. Set up the environment with obstacles and terrain
6. Configure ROS connection and message publishing
7. Test the scene for performance and accuracy

## Troubleshooting

### Physics Issues
- Ensure all robot parts have proper colliders
- Check mass distribution is realistic
- Verify joint limits and constraints

### Performance Issues
- Reduce polygon count in complex meshes
- Use occlusion culling for large environments
- Limit physics update rate to match real-time requirements

### ROS Connection Issues
- Verify TCP connection settings
- Check ROS network configuration
- Ensure message types match between Unity and ROS nodes

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unity Robotics Package Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)
- [Unity Asset Store - Robotics Assets](https://assetstore.unity.com/)