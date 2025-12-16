#!/usr/bin/env python3
"""
ROS 2 Sensor Configuration Examples

This file contains examples of configuring various types of sensors in ROS 2,
including IMU, LiDAR, cameras, and joint encoders.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, JointState, CameraInfo, Image
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import math
import numpy as np
from cv_bridge import CvBridge
import cv2


class ImuSensorConfig(Node):
    """
    Example of IMU sensor configuration and publishing
    """
    def __init__(self):
        super().__init__('imu_sensor_config')

        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Timer to publish IMU data at 100Hz
        self.imu_timer = self.create_timer(0.01, self.publish_imu_data)

        # Simulated IMU state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.time = 0.0

        self.get_logger().info('IMU sensor configured')

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate orientation (convert Euler to quaternion)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Add some noise to make it more realistic
        noise = 0.01
        msg.orientation_covariance = [noise, 0, 0, 0, noise, 0, 0, 0, noise]

        # Simulate angular velocity (derivative of orientation)
        msg.angular_velocity.x = 0.1 * math.sin(self.time * 2.0)
        msg.angular_velocity.y = 0.05 * math.cos(self.time * 1.5)
        msg.angular_velocity.z = 0.2 * math.sin(self.time * 3.0)

        # Simulate linear acceleration (gravity + movement)
        msg.linear_acceleration.x = 9.81 * math.sin(self.time * 0.5)
        msg.linear_acceleration.y = 9.81 * math.cos(self.time * 0.7)
        msg.linear_acceleration.z = 9.81 + 0.5 * math.sin(self.time * 1.2)

        # Publish the message
        self.imu_pub.publish(msg)

        # Update simulation state
        self.roll += 0.01 * math.sin(self.time)
        self.pitch += 0.005 * math.cos(self.time)
        self.yaw += 0.02 * math.sin(self.time * 0.8)
        self.time += 0.01


class LidarSensorConfig(Node):
    """
    Example of LiDAR sensor configuration and publishing
    """
    def __init__(self):
        super().__init__('lidar_sensor_config')

        # Publisher for LiDAR data
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer to publish LiDAR data at 10Hz
        self.lidar_timer = self.create_timer(0.1, self.publish_lidar_data)

        # LiDAR configuration
        self.angle_min = -math.pi / 2  # -90 degrees
        self.angle_max = math.pi / 2   # 90 degrees
        self.angle_increment = math.pi / 180  # 1 degree per increment
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 10.0
        self.time_increment = 0.001

        self.get_logger().info('LiDAR sensor configured')

    def publish_lidar_data(self):
        """Publish simulated LiDAR data"""
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Calculate number of ranges
        num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Generate simulated ranges (with some obstacles)
        ranges = []
        for i in range(num_ranges):
            angle = self.angle_min + i * self.angle_increment

            # Simulate some obstacles at different distances
            distance = self.range_max  # Default to max range (no obstacle)

            # Add some simulated obstacles
            if abs(angle) < 0.2:  # Front
                distance = 2.0 + 0.5 * math.sin(self.get_clock().now().nanoseconds / 1e9 * 2)
            elif abs(angle - 0.5) < 0.1:  # Right side
                distance = 1.5
            elif abs(angle + 0.5) < 0.1:  # Left side
                distance = 3.0

            # Add some noise
            distance += np.random.normal(0, 0.02)

            # Clamp to valid range
            distance = max(self.range_min, min(self.range_max, distance))
            ranges.append(distance)

        msg.ranges = ranges
        msg.intensities = [100.0] * len(ranges)  # Simulated intensity

        # Publish the message
        self.lidar_pub.publish(msg)


class JointSensorConfig(Node):
    """
    Example of joint sensor configuration and publishing
    """
    def __init__(self):
        super().__init__('joint_sensor_config')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish joint data at 50Hz
        self.joint_timer = self.create_timer(0.02, self.publish_joint_data)

        # Joint names and initial positions
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint'
        ]

        # Initialize positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        # Time for simulation
        self.time = 0.0

        self.get_logger().info('Joint sensor configured')

    def publish_joint_data(self):
        """Publish simulated joint state data"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Update joint positions with simulated movement
        for i, name in enumerate(self.joint_names):
            # Different movement patterns for different joint types
            if 'hip' in name:
                self.joint_positions[i] = 0.2 * math.sin(self.time * 0.5 + i)
            elif 'knee' in name:
                self.joint_positions[i] = 0.3 * math.sin(self.time * 0.7 + i)
            elif 'ankle' in name:
                self.joint_positions[i] = 0.1 * math.sin(self.time * 0.9 + i)
            elif 'shoulder' in name:
                self.joint_positions[i] = 0.4 * math.sin(self.time * 0.4 + i)
            elif 'elbow' in name:
                self.joint_positions[i] = 0.5 * math.sin(self.time * 0.6 + i)
            elif 'wrist' in name:
                self.joint_positions[i] = 0.3 * math.sin(self.time * 0.8 + i)

            # Calculate velocities (derivative of position)
            dt = 0.02  # Timer period
            if dt > 0:
                self.joint_velocities[i] = (
                    (self.joint_positions[i] -
                     0.2 * math.sin((self.time - dt) * (0.5 if 'hip' in name else 0.7 if 'knee' in name else 0.4) + i))
                    / dt
                )
            else:
                self.joint_velocities[i] = 0.0

        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        # Publish the message
        self.joint_pub.publish(msg)

        # Update simulation time
        self.time += 0.02


class CameraSensorConfig(Node):
    """
    Example of camera sensor configuration and publishing
    """
    def __init__(self):
        super().__init__('camera_sensor_config')

        # Publishers for camera data
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Timer to publish camera data at 30Hz
        self.camera_timer = self.create_timer(1.0/30.0, self.publish_camera_data)

        # Camera configuration
        self.width = 640
        self.height = 480
        self.fps = 30

        # Create OpenCV bridge
        self.bridge = CvBridge()

        # Camera info message
        self.camera_info_msg = self.create_camera_info()

        self.get_logger().info('Camera sensor configured')

    def create_camera_info(self):
        """Create camera info message"""
        msg = CameraInfo()
        msg.header.frame_id = 'camera_link'
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'

        # Intrinsic parameters (example values)
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
        msg.k = [320.0, 0.0, 320.0,  # fx, 0, cx
                 0.0, 240.0, 240.0,  # 0, fy, cy
                 0.0, 0.0, 1.0]      # 0, 0, 1
        msg.r = [1.0, 0.0, 0.0,      # R: Rectification matrix (identity)
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [320.0, 0.0, 320.0, 0.0,  # P: Projection matrix
                 0.0, 240.0, 240.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    def publish_camera_data(self):
        """Publish simulated camera data"""
        # Create a simulated image (with some patterns)
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add some simulated features
        center_x, center_y = self.width // 2, self.height // 2
        time_offset = self.get_clock().now().nanoseconds / 1e9

        # Draw a moving circle
        circle_radius = 50
        circle_x = int(center_x + 100 * math.cos(time_offset))
        circle_y = int(center_y + 50 * math.sin(time_offset * 0.7))
        cv2.circle(img, (circle_x, circle_y), circle_radius, (0, 255, 0), -1)

        # Draw some lines
        cv2.line(img, (0, center_y), (self.width, center_y), (255, 0, 0), 2)
        cv2.line(img, (center_x, 0), (center_x, self.height), (255, 0, 0), 2)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_link'

        # Publish image and camera info
        self.image_pub.publish(ros_image)

        self.camera_info_msg.header.stamp = ros_image.header.stamp
        self.info_pub.publish(self.camera_info_msg)


class SensorFusionNode(Node):
    """
    Example of combining multiple sensor types
    """
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribe to various sensors
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(JointState, 'fused_sensor_data', 10)

        # Store latest sensor data
        self.last_imu = None
        self.last_lidar = None
        self.last_joints = None
        self.last_camera = None

        self.get_logger().info('Sensor fusion node initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.last_imu = msg

    def lidar_callback(self, msg):
        """Handle LiDAR data"""
        self.last_lidar = msg

    def joint_callback(self, msg):
        """Handle joint state data"""
        self.last_joints = msg

    def camera_callback(self, msg):
        """Handle camera data"""
        self.last_camera = msg

    def create_fused_data(self):
        """Create fused sensor data message"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # For this example, we'll create some derived values
        # based on the sensor data
        if self.last_imu:
            # Extract some values from IMU
            msg.name.append('imu_roll')
            msg.position.append(2 * math.asin(self.last_imu.orientation.z))
            msg.velocity.append(self.last_imu.angular_velocity.x)
            msg.effort.append(self.last_imu.linear_acceleration.x)

        if self.last_joints and len(self.last_joints.position) > 0:
            # Add some joint-based derived values
            avg_joint_pos = sum(self.last_joints.position) / len(self.last_joints.position)
            msg.name.extend(['avg_joint_pos', 'joint_count'])
            msg.position.extend([avg_joint_pos, float(len(self.last_joints.position))])
            msg.velocity.extend([0.0, 0.0])
            msg.effort.extend([0.0, 0.0])

        return msg


def main(args=None):
    """
    Main function to demonstrate sensor configuration examples
    """
    rclpy.init(args=args)

    # Create sensor configuration nodes
    imu_node = ImuSensorConfig()
    lidar_node = LidarSensorConfig()
    joint_node = JointSensorConfig()
    camera_node = CameraSensorConfig()
    fusion_node = SensorFusionNode()

    print("Starting sensor configuration examples...")
    print("IMU node simulates IMU sensor data")
    print("LiDAR node simulates LiDAR sensor data")
    print("Joint node simulates joint encoder data")
    print("Camera node simulates camera sensor data")
    print("Fusion node combines multiple sensor inputs")

    try:
        # Create multi-threaded executor to handle all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(imu_node)
        executor.add_node(lidar_node)
        executor.add_node(joint_node)
        executor.add_node(camera_node)
        executor.add_node(fusion_node)

        print("All sensor configuration nodes started. Press Ctrl+C to stop.")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down sensor configuration nodes...")
    finally:
        # Clean up all nodes
        imu_node.destroy_node()
        lidar_node.destroy_node()
        joint_node.destroy_node()
        camera_node.destroy_node()
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()