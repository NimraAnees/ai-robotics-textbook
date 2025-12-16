#!/usr/bin/env python3

"""
Humanoid Navigation Controller

This script implements a navigation controller for humanoid robots
that integrates with Isaac ROS and Navigation 2 for autonomous navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import math
from collections import deque


class HumanoidNavigationController(Node):
    """
    Humanoid Navigation Controller

    This node implements navigation algorithms for humanoid robots,
    including path planning, obstacle avoidance, and locomotion control.
    """

    def __init__(self):
        super().__init__('humanoid_navigation_controller')

        # Declare parameters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('min_distance_to_obstacle', 0.5)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('goal_tolerance', 0.2)

        # Get parameter values
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.min_distance_to_obstacle = self.get_parameter('min_distance_to_obstacle').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            sensor_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.map_sub = self.create_subscription(
            PointCloud2,
            '/visual_slam/map',
            self.map_callback,
            sensor_qos
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/navigation/path',
            10
        )

        self.local_plan_pub = self.create_publisher(
            Path,
            '/navigation/local_plan',
            10
        )

        self.viz_pub = self.create_publisher(
            MarkerArray,
            '/navigation/visualization',
            10
        )

        # Initialize navigation state
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.current_velocity = np.array([0.0, 0.0])  # linear, angular
        self.goal_pose = None
        self.laser_data = None
        self.obstacles = []
        self.path = []
        self.local_path = []
        self.is_navigating = False
        self.safety_mode = False

        # Navigation history for smoothing
        self.pose_history = deque(maxlen=10)

        # Create navigation timer
        self.nav_timer = self.create_timer(0.1, self.navigation_callback)

        self.get_logger().info('Humanoid Navigation Controller initialized')

    def odom_callback(self, msg):
        """Process odometry data to update current pose"""
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (convert quaternion to euler)
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Update current pose
        self.current_pose = np.array([x, y, theta])

        # Store in history for smoothing
        self.pose_history.append([x, y, theta])

        # Extract velocity
        linear_vel = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        angular_vel = msg.twist.twist.angular.z
        self.current_velocity = np.array([linear_vel, angular_vel])

    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.laser_data = msg
        self.obstacles = self.process_laser_scan(msg)

    def map_callback(self, msg):
        """Process map data from SLAM"""
        # Process PointCloud2 map data
        # This is a simplified implementation
        self.get_logger().debug('Received map data')

    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.goal_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            0.0  # We'll calculate the desired orientation
        ])

        self.get_logger().info(f'New goal received: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})')

        # Start navigation
        self.is_navigating = True
        self.calculate_path()

    def process_laser_scan(self, scan_msg):
        """Process laser scan to detect obstacles"""
        obstacles = []

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        for i, range_val in enumerate(scan_msg.ranges):
            if not (float('inf') == range_val or float('-inf') == range_val or math.isnan(range_val)):
                if range_val < self.min_distance_to_obstacle + self.robot_radius:
                    angle = angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    obstacles.append((x, y, range_val))

        return obstacles

    def calculate_path(self):
        """Calculate global path to goal (simplified A* or Dijkstra)"""
        if self.goal_pose is None:
            return

        # For this example, we'll create a simple direct path
        # In practice, this would implement A*, Dijkstra, or other path planning algorithms
        start = self.current_pose[:2]
        goal = self.goal_pose[:2]

        # Calculate direct path
        direction = goal - start
        distance = np.linalg.norm(direction)

        if distance > 0:
            step_size = 0.1  # 10cm steps
            num_steps = int(distance / step_size)

            self.path = []
            for i in range(num_steps + 1):
                t = i / num_steps if num_steps > 0 else 0
                point = start + t * direction
                self.path.append(point)

            # Publish the path
            self.publish_path(self.path, self.path_pub)

    def calculate_local_path(self):
        """Calculate local path considering obstacles"""
        if not self.path or not self.obstacles:
            return self.path[:10]  # Return first 10 points of global path

        # Implement local path planning with obstacle avoidance
        # This is a simplified version using vector field histogram or similar
        current_pos = self.current_pose[:2]

        # Find closest point on global path
        closest_idx = 0
        min_dist = float('inf')
        for i, point in enumerate(self.path):
            dist = np.linalg.norm(current_pos - point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Create local path from current position to points ahead
        local_path = []
        start_idx = min(closest_idx, len(self.path) - 1)

        for i in range(start_idx, min(start_idx + 20, len(self.path))):
            local_path.append(self.path[i])

        # Adjust path based on obstacles
        adjusted_path = self.avoid_obstacles_in_path(local_path)

        self.local_path = adjusted_path
        self.publish_path(adjusted_path, self.local_plan_pub)

    def avoid_obstacles_in_path(self, path):
        """Adjust path to avoid obstacles"""
        if not path or not self.obstacles:
            return path

        adjusted_path = []
        current_pos = self.current_pose[:2]

        for point in path:
            # Check if path segment has obstacles
            collision = False
            for obs_x, obs_y, _ in self.obstacles:
                # Simple distance check to path segment
                obs_point = np.array([obs_x, obs_y])
                dist_to_segment = self.distance_point_to_segment(
                    current_pos, point, obs_point
                )

                if dist_to_segment < self.robot_radius:
                    collision = True
                    break

            if collision:
                # Simple obstacle avoidance - move around the obstacle
                # In practice, this would use more sophisticated algorithms
                adjusted_point = point + np.array([
                    np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.2, 0.2)
                ])
                adjusted_path.append(adjusted_point)
            else:
                adjusted_path.append(point)

            current_pos = point

        return adjusted_path

    def distance_point_to_segment(self, seg_start, seg_end, point):
        """Calculate distance from point to line segment"""
        seg_vec = seg_end - seg_start
        point_vec = point - seg_start
        seg_len = np.linalg.norm(seg_vec)

        if seg_len == 0:
            return np.linalg.norm(point - seg_start)

        seg_unit_vec = seg_vec / seg_len
        proj_len = np.dot(point_vec, seg_unit_vec)

        if proj_len <= 0:
            return np.linalg.norm(point - seg_start)
        elif proj_len >= seg_len:
            return np.linalg.norm(point - seg_end)
        else:
            proj_point = seg_start + proj_len * seg_unit_vec
            return np.linalg.norm(point - proj_point)

    def navigation_callback(self):
        """Main navigation control loop"""
        if not self.is_navigating or self.goal_pose is None:
            # Stop robot if not navigating
            self.publish_velocity(0.0, 0.0)
            return

        # Check if goal is reached
        current_pos = self.current_pose[:2]
        goal_pos = self.goal_pose[:2]
        distance_to_goal = np.linalg.norm(current_pos - goal_pos)

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.is_navigating = False
            self.publish_velocity(0.0, 0.0)
            return

        # Calculate local path
        self.calculate_local_path()

        # Determine next waypoint
        next_waypoint = self.get_next_waypoint()
        if next_waypoint is None:
            self.publish_velocity(0.0, 0.0)
            return

        # Calculate control commands
        linear_vel, angular_vel = self.calculate_control(next_waypoint)

        # Check for safety
        if self.check_safety():
            # Enter safety mode
            self.safety_mode = True
            self.publish_velocity(0.0, 0.0)
            self.get_logger().warn('Safety stop: obstacle too close!')
            return
        else:
            self.safety_mode = False

        # Publish velocity commands
        self.publish_velocity(linear_vel, angular_vel)

        # Publish visualization
        self.publish_visualization()

    def get_next_waypoint(self):
        """Get the next waypoint from local path"""
        if not self.local_path:
            return None

        current_pos = self.current_pose[:2]

        # Find the closest waypoint ahead
        for waypoint in self.local_path:
            if np.linalg.norm(current_pos - waypoint) > self.goal_tolerance:
                return waypoint

        # If no suitable waypoint, return the last one
        return self.local_path[-1] if self.local_path else None

    def calculate_control(self, waypoint):
        """Calculate linear and angular velocity to reach waypoint"""
        current_pos = self.current_pose[:2]
        current_theta = self.current_pose[2]

        # Calculate desired direction
        direction = waypoint - current_pos
        distance = np.linalg.norm(direction)

        if distance < 0.01:  # Very close to waypoint
            return 0.0, 0.0

        # Calculate desired heading
        desired_theta = math.atan2(direction[1], direction[0])

        # Calculate angle difference
        angle_diff = desired_theta - current_theta
        # Normalize angle to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Simple proportional controller
        angular_kp = 1.0
        linear_kp = 0.5

        # Calculate velocities
        angular_vel = angular_kp * angle_diff
        linear_vel = min(linear_kp * distance, self.max_linear_velocity)

        # Limit velocities
        angular_vel = max(min(angular_vel, self.max_angular_velocity), -self.max_angular_velocity)

        return linear_vel, angular_vel

    def check_safety(self):
        """Check if it's safe to continue navigation"""
        if not self.obstacles:
            return False

        # Check for obstacles too close in front of robot
        for obs_x, obs_y, _ in self.obstacles:
            # Convert to robot's frame
            distance = math.sqrt(obs_x**2 + obs_y**2)
            if distance < self.min_distance_to_obstacle:
                # Check if obstacle is in front of robot (within 90 degrees)
                angle_to_obstacle = math.atan2(obs_y, obs_x)
                if abs(angle_to_obstacle) < math.pi / 4:  # 45 degrees
                    return True

        return False

    def publish_velocity(self, linear, angular):
        """Publish velocity commands to robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.cmd_vel_pub.publish(cmd_msg)

    def publish_path(self, path_points, publisher):
        """Publish path as Path message"""
        if not path_points:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        publisher.publish(path_msg)

    def publish_visualization(self):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()

        # Goal marker
        if self.goal_pose is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'navigation'
            goal_marker.id = 0
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.goal_pose[0]
            goal_marker.pose.position.y = self.goal_pose[1]
            goal_marker.pose.position.z = 0.0
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.a = 1.0
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 0.0
            marker_array.markers.append(goal_marker)

        # Robot path marker
        if len(self.path) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'navigation'
            path_marker.id = 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.05
            path_marker.color.a = 1.0
            path_marker.color.r = 0.0
            path_marker.color.g = 0.0
            path_marker.color.b = 1.0

            for point in self.path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                path_marker.points.append(p)

            marker_array.markers.append(path_marker)

        # Local path marker
        if len(self.local_path) > 1:
            local_path_marker = Marker()
            local_path_marker.header.frame_id = 'map'
            local_path_marker.header.stamp = self.get_clock().now().to_msg()
            local_path_marker.ns = 'navigation'
            local_path_marker.id = 2
            local_path_marker.type = Marker.LINE_STRIP
            local_path_marker.action = Marker.ADD
            local_path_marker.scale.x = 0.03
            local_path_marker.color.a = 1.0
            local_path_marker.color.r = 1.0
            local_path_marker.color.g = 0.0
            local_path_marker.color.b = 1.0

            for point in self.local_path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                local_path_marker.points.append(p)

            marker_array.markers.append(local_path_marker)

        self.viz_pub.publish(marker_array)

    def stop_robot(self):
        """Stop the robot immediately"""
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    """Main function to run the navigation controller"""
    rclpy.init(args=args)

    nav_controller = HumanoidNavigationController()

    try:
        rclpy.spin(nav_controller)
    except KeyboardInterrupt:
        nav_controller.get_logger().info('Shutting down navigation controller...')
    finally:
        nav_controller.stop_robot()
        nav_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()