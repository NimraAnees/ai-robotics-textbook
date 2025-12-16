#!/usr/bin/env python3

"""
Isaac ROS Visual SLAM Node Example

This script demonstrates the usage of Isaac ROS Visual SLAM package
for real-time mapping and localization on a humanoid robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2
from cv_bridge import CvBridge


class IsaacROSVisualSLAMNode(Node):
    """
    Isaac ROS Visual SLAM Node

    This node interfaces with Isaac ROS Visual SLAM package
    to provide real-time mapping and localization.
    """

    def __init__(self):
        super().__init__('isaac_ros_vslam_node')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('enable_imu_fusion', True)
        self.declare_parameter('min_distance_between_poses', 0.1)
        self.declare_parameter('publish_tf', True)

        # Get parameter values
        self.enable_imu_fusion = self.get_parameter('enable_imu_fusion').value
        self.min_distance_between_poses = self.get_parameter('min_distance_between_poses').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for camera and IMU data
        self.left_image_sub = self.create_subscription(
            Image,
            '/camera/left/image_rect_color',
            self.left_image_callback,
            sensor_qos
        )

        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/right/image_rect_color',
            self.right_image_callback,
            sensor_qos
        )

        if self.enable_imu_fusion:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data',
                self.imu_callback,
                10
            )

        # Create publishers for SLAM outputs
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/visual_slam/odometry',
            10
        )

        self.map_publisher = self.create_publisher(
            MarkerArray,
            '/visual_slam/map',
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/visual_slam/pose',
            10
        )

        # Initialize SLAM state variables
        self.latest_left_image = None
        self.latest_right_image = None
        self.latest_imu_data = None
        self.current_pose = np.eye(4)  # 4x4 identity transformation matrix
        self.map_points = []

        # Log initialization
        self.get_logger().info('Isaac ROS Visual SLAM Node initialized')
        self.get_logger().info(f'IMU fusion enabled: {self.enable_imu_fusion}')
        self.get_logger().info(f'Min distance between poses: {self.min_distance_between_poses}')

    def left_image_callback(self, msg):
        """Process left camera image for stereo SLAM"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store the image for stereo processing
            self.latest_left_image = cv_image

            # Process stereo pair if both images are available
            if self.latest_right_image is not None:
                self.process_stereo_pair(self.latest_left_image, self.latest_right_image)

        except Exception as e:
            self.get_logger().error(f'Error processing left image: {e}')

    def right_image_callback(self, msg):
        """Process right camera image for stereo SLAM"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store the image for stereo processing
            self.latest_right_image = cv_image

            # Process stereo pair if both images are available
            if self.latest_left_image is not None:
                self.process_stereo_pair(self.latest_left_image, self.latest_right_image)

        except Exception as e:
            self.get_logger().error(f'Error processing right image: {e}')

    def imu_callback(self, msg):
        """Process IMU data for pose refinement"""
        # Store IMU data for fusion with visual data
        self.latest_imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def process_stereo_pair(self, left_image, right_image):
        """Process stereo image pair for depth estimation and feature tracking"""
        try:
            # Create stereo matcher (using OpenCV SGBM as example)
            stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=64,
                blockSize=11,
                P1=8 * 3 * 11**2,
                P2=32 * 3 * 11**2,
                disp12MaxDiff=1,
                uniquenessRatio=15,
                speckleWindowSize=0,
                speckleRange=2,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )

            # Compute disparity map
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

            # Convert disparity to depth
            baseline = 0.12  # Example baseline in meters
            focal_length = 640  # Example focal length in pixels
            depth_map = (baseline * focal_length) / (disparity + 1e-6)

            # Feature detection and tracking for SLAM
            orb = cv2.ORB_create(nfeatures=1000)
            kp1, des1 = orb.detectAndCompute(left_image, None)

            # Use KLT tracker or similar for feature tracking
            if hasattr(self, 'prev_image'):
                # Lucas-Kanade optical flow for feature tracking
                prev_kp = np.float32([kp.pt for kp in self.prev_kp]).reshape(-1, 1, 2)
                curr_kp, status, err = cv2.calcOpticalFlowPyrLK(
                    self.prev_image, left_image, prev_kp, None
                )

                # Select good points
                good_new = curr_kp[status == 1]
                good_prev = prev_kp[status == 1]

                # Estimate motion between frames
                if len(good_new) >= 8:  # Need at least 8 points for essential matrix
                    E, mask = cv2.findEssentialMat(
                        good_new, good_prev, focal=focal_length,
                        pp=(left_image.shape[1]//2, left_image.shape[0]//2),
                        method=cv2.RANSAC, threshold=1.0
                    )

                    if E is not None:
                        # Recover pose from essential matrix
                        _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_prev)

                        # Update current pose
                        transformation = np.eye(4)
                        transformation[:3, :3] = R
                        transformation[:3, 3] = t.flatten()

                        # Check if we should update pose based on minimum distance
                        translation_norm = np.linalg.norm(t)
                        if translation_norm > self.min_distance_between_poses:
                            self.current_pose = self.current_pose @ transformation

                            # Publish updated pose
                            self.publish_pose()
                            self.publish_odometry()

            # Store current image and keypoints for next iteration
            self.prev_image = left_image.copy()
            self.prev_kp = kp1

        except Exception as e:
            self.get_logger().error(f'Error processing stereo pair: {e}')

    def publish_pose(self):
        """Publish current robot pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation from transformation matrix
        position = self.current_pose[:3, 3]
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]

        # Convert rotation matrix to quaternion
        rotation_matrix = self.current_pose[:3, :3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.pose_publisher.publish(pose_msg)

    def publish_odometry(self):
        """Publish odometry information"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Set position from current pose
        position = self.current_pose[:3, 3]
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]

        # Set orientation from current pose
        rotation_matrix = self.current_pose[:3, :3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(rotation_matrix)
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        # TODO: Estimate velocity from pose changes
        # This is a simplified example - in practice, you'd calculate velocities
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom_msg)

    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a rotation matrix to quaternion.

        Args:
            R: 3x3 rotation matrix

        Returns:
            (qw, qx, qy, qz): Quaternion components
        """
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return qw, qx, qy, qz


def main(args=None):
    """Main function to run the Isaac ROS Visual SLAM node"""
    rclpy.init(args=args)

    vslam_node = IsaacROSVisualSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()