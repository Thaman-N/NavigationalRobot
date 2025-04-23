#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
from cv_bridge import CvBridge
import tf_transformations


class PerceptionNode(Node):
    """
    Node for processing sensor data to detect elements relevant to ethical decisions.
    In a real robot, this would use computer vision and other perception algorithms.
    For this simulation, we'll simulate detection based on known positions.
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters
        self.declare_parameter('area1_center_x', 12.0)
        self.declare_parameter('area1_center_y', 6.0)
        self.declare_parameter('area2_center_x', 10.0)
        self.declare_parameter('area2_center_y', -5.0)
        self.declare_parameter('area1_radius', 3.0)
        self.declare_parameter('area2_radius', 3.0)
        self.declare_parameter('decision_point_x', 5.0)
        self.declare_parameter('decision_point_y', 0.0)
        self.declare_parameter('decision_radius', 1.0)
        
        # Get parameters
        self.area1_center = [
            self.get_parameter('area1_center_x').get_parameter_value().double_value,
            self.get_parameter('area1_center_y').get_parameter_value().double_value
        ]
        self.area2_center = [
            self.get_parameter('area2_center_x').get_parameter_value().double_value,
            self.get_parameter('area2_center_y').get_parameter_value().double_value
        ]
        self.area1_radius = self.get_parameter('area1_radius').get_parameter_value().double_value
        self.area2_radius = self.get_parameter('area2_radius').get_parameter_value().double_value
        self.decision_point = [
            self.get_parameter('decision_point_x').get_parameter_value().double_value,
            self.get_parameter('decision_point_y').get_parameter_value().double_value
        ]
        self.decision_radius = self.get_parameter('decision_radius').get_parameter_value().double_value
        
        # Publishers
        self.area1_people_pub = self.create_publisher(Int32, 'area1/people_count', 10)
        self.area2_people_pub = self.create_publisher(Int32, 'area2/people_count', 10)
        self.area1_time_pub = self.create_publisher(Float32, 'area1/estimated_time', 10)
        self.area2_time_pub = self.create_publisher(Float32, 'area2/estimated_time', 10)
        self.dilemma_type_pub = self.create_publisher(String, 'dilemma_type', 10)
        self.at_decision_point_pub = self.create_publisher(String, 'at_decision_point', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        
        # State
        self.robot_position = None
        self.robot_orientation = None
        self.obstacles = []
        self.people_detected = {
            'area1': [],
            'area2': []
        }
        
        # Simulated "ground truth" - in a real system this would come from perception
        # For this simulation, we'll hardcode these values based on the world file
        self.area1_people_count = 5  # From the world file
        self.area2_people_count = 2  # From the world file
        
        # Area 1 has an obstacle, making rescue harder and time longer
        self.area1_time_estimate = 12.0  # seconds
        # Area 2 is more accessible, making rescue faster
        self.area2_time_estimate = 5.0   # seconds
        
        # The current dilemma type
        self.current_dilemma = "PEOPLE_VS_TIME"
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Timer for publishing simulated perception data
        self.perception_timer = self.create_timer(0.5, self.publish_perception_data)
        
        self.get_logger().info('Perception Node initialized')
    
    def scan_callback(self, msg):
        """Process LiDAR scan data to detect obstacles."""
        # In a real system, we'd process the scan to detect obstacles
        # For simulation, we'll use the known positions
        
        # Example of how to process scan data:
        # Extract ranges from the LiDAR scan
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to cartesian coordinates relative to the robot
        x_local = valid_ranges * np.cos(valid_angles)
        y_local = valid_ranges * np.sin(valid_angles)
        
        # We could transform these to global coordinates if needed
        # For now, we'll just store them for visualization or obstacle avoidance
        self.obstacles = list(zip(x_local, y_local))
    
    def odom_callback(self, msg):
        """Process odometry data to track robot position."""
        # Extract position
        self.robot_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        
        # Extract orientation quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        euler = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        self.robot_orientation = euler[2]  # Yaw (rotation around z-axis)
        
        # Check if robot is at decision point
        if self.is_at_decision_point():
            msg = String()
            msg.data = "true"
            self.at_decision_point_pub.publish(msg)
            self.get_logger().info('Robot is at decision point')
    
    def camera_callback(self, msg):
        """Process camera data to detect people and obstacles."""
        # In a real system, we'd use computer vision to detect people
        # For simulation, we'll use the known positions
        
        # Example of how to process image data:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Here we would run computer vision algorithms
            # For simulation, we'll use the known counts
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def is_at_decision_point(self):
        """Check if the robot is at the decision point."""
        if self.robot_position is None:
            return False
        
        distance = math.sqrt(
            (self.robot_position[0] - self.decision_point[0])**2 +
            (self.robot_position[1] - self.decision_point[1])**2
        )
        
        return distance < self.decision_radius
    
    def publish_perception_data(self):
        """Publish simulated perception data."""
        # Publish people count for each area
        area1_people_msg = Int32()
        area1_people_msg.data = self.area1_people_count
        self.area1_people_pub.publish(area1_people_msg)
        
        area2_people_msg = Int32()
        area2_people_msg.data = self.area2_people_count
        self.area2_people_pub.publish(area2_people_msg)
        
        # Publish time estimates for each area
        area1_time_msg = Float32()
        area1_time_msg.data = self.area1_time_estimate
        self.area1_time_pub.publish(area1_time_msg)
        
        area2_time_msg = Float32()
        area2_time_msg.data = self.area2_time_estimate
        self.area2_time_pub.publish(area2_time_msg)
        
        # Publish dilemma type
        dilemma_msg = String()
        dilemma_msg.data = self.current_dilemma
        self.dilemma_type_pub.publish(dilemma_msg)
        
        # In a more complex simulation, we could update these values dynamically
        # based on the robot's perception and world state


def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()