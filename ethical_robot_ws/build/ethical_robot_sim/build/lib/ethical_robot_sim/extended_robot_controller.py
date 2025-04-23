#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from enum import Enum, auto
import tf_transformations
import time


class RobotState(Enum):
    """Extended states for the robot's state machine."""
    NAVIGATING_TO_DECISION = auto()  # Moving to the first decision point
    WAITING_FOR_DECISION = auto()    # At decision point, waiting for ethical decision
    NAVIGATING_TO_AREA1 = auto()     # Moving to area 1 based on decision
    NAVIGATING_TO_AREA2 = auto()     # Moving to area 2 based on decision
    AT_SECONDARY_DECISION = auto()   # At the secondary decision point
    NAVIGATING_TO_AREA3 = auto()     # Moving to the new area 3
    NAVIGATING_TO_AREA4 = auto()     # Moving to the new area 4
    RESCUE_OPERATION = auto()        # Performing rescue in the chosen area
    RETURNING_TO_BASE = auto()       # Returning to starting position
    MISSION_COMPLETE = auto()        # Mission completed


class ExtendedRobotController(Node):
    """Extended controller for the robot's movement with more decision points and areas."""
    
    def __init__(self):
        super().__init__('extended_robot_controller')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('decision_point_x', 5.0)
        self.declare_parameter('decision_point_y', 0.0)
        self.declare_parameter('area1_x', 14.0)
        self.declare_parameter('area1_y', 8.0)
        self.declare_parameter('area2_x', 13.0)
        self.declare_parameter('area2_y', -8.0)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        
        self.decision_point = [
            self.get_parameter('decision_point_x').get_parameter_value().double_value,
            self.get_parameter('decision_point_y').get_parameter_value().double_value
        ]
        
        self.area1_position = [
            self.get_parameter('area1_x').get_parameter_value().double_value,
            self.get_parameter('area1_y').get_parameter_value().double_value
        ]
        
        self.area2_position = [
            self.get_parameter('area2_x').get_parameter_value().double_value,
            self.get_parameter('area2_y').get_parameter_value().double_value
        ]
        
        # New positions for additional areas - updated with increased distances
        self.area3_position = [16.0, 3.0]  # Hospital area
        self.area4_position = [9.0, 12.0]  # School area
        
        # Secondary decision points - positioned along the paths to Area 1 and Area 2
        self.secondary_decision_point_area1 = [9.0, 4.0]  # Secondary decision on way to Area 1
        self.secondary_decision_point_area2 = [9.0, -4.0]  # Secondary decision on way to Area 2
        
        # Positions for starting point (for return journey)
        self.start_position = [0.0, 0.0]
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        self.secondary_decision_pub = self.create_publisher(String, 'at_secondary_decision', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.decision_sub = self.create_subscription(
            String, 'ethical_decision', self.decision_callback, 10)
        self.at_decision_point_sub = self.create_subscription(
            String, 'at_decision_point', self.at_decision_point_callback, 10)
        self.reasoning_sub = self.create_subscription(
            String, 'decision_reasoning', self.reasoning_callback, 10)
        self.secondary_choice_sub = self.create_subscription(
            String, 'secondary_choice', self.secondary_choice_callback, 10)
        
        # State variables
        self.robot_position = None
        self.robot_orientation = None
        self.current_state = RobotState.NAVIGATING_TO_DECISION
        self.ethical_decision = None
        self.secondary_choice = None
        self.decision_reasoning = None
        self.goal_position = self.decision_point
        self.at_decision_point = False
        self.obstacle_detected = False
        self.obstacle_directions = []
        self.secondary_decision_point = None
        self.current_operation_start_time = None
        
        # Create a timer for the control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Extended Robot Controller initialized')
        self.publish_state()
    
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
        
        # Check if robot is at a secondary decision point
        if (self.current_state == RobotState.NAVIGATING_TO_AREA1 and 
            self.is_at_position(self.secondary_decision_point_area1)):
            self.handle_secondary_decision_point()
        elif (self.current_state == RobotState.NAVIGATING_TO_AREA2 and 
            self.is_at_position(self.secondary_decision_point_area2)):
            self.handle_secondary_decision_point()
    
    def scan_callback(self, msg):
        """Process LiDAR scan data for obstacle detection."""
        # Extract ranges from the LiDAR scan
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Check for obstacles in front of the robot
        front_indices = np.logical_and(angles > -0.5, angles < 0.5)
        front_ranges = ranges[front_indices]
        
        # Check if any obstacle is too close
        self.obstacle_detected = np.any(front_ranges < self.obstacle_threshold)
        
        # If obstacle detected, determine possible avoidance directions
        if self.obstacle_detected:
            # Find direction with most free space
            valid_indices = np.isfinite(ranges)
            valid_ranges = ranges[valid_indices]
            valid_angles = angles[valid_indices]
            
            # Find the angle with maximum range
            if len(valid_ranges) > 0:
                max_range_idx = np.argmax(valid_ranges)
                self.obstacle_directions = [valid_angles[max_range_idx]]
                self.get_logger().debug(f'Obstacle detected, turning to {self.obstacle_directions[0]}')
    
    def decision_callback(self, msg):
        """Receive the ethical decision."""
        self.ethical_decision = msg.data
        self.get_logger().info(f'Received ethical decision: {self.ethical_decision}')
        
        # Update the goal based on the decision
        if self.current_state == RobotState.WAITING_FOR_DECISION:
            if self.ethical_decision == "area1":
                self.current_state = RobotState.NAVIGATING_TO_AREA1
                self.goal_position = self.secondary_decision_point_area1
                self.secondary_decision_point = self.secondary_decision_point_area1
            elif self.ethical_decision == "area2":
                self.current_state = RobotState.NAVIGATING_TO_AREA2
                self.goal_position = self.secondary_decision_point_area2
                self.secondary_decision_point = self.secondary_decision_point_area2
            else:
                self.get_logger().warning(f'Unknown decision: {self.ethical_decision}')
            
            self.publish_state()
    
    def secondary_choice_callback(self, msg):
        """Receive the decision for the secondary choice."""
        self.secondary_choice = msg.data
        self.get_logger().info(f'Received secondary choice: {self.secondary_choice}')
        
        # Update goal based on secondary choice
        if self.current_state == RobotState.AT_SECONDARY_DECISION:
            if self.secondary_choice == "continue":
                # Continue to the original destination
                if self.ethical_decision == "area1":
                    self.current_state = RobotState.NAVIGATING_TO_AREA1
                    self.goal_position = self.area1_position
                else:  # area2
                    self.current_state = RobotState.NAVIGATING_TO_AREA2
                    self.goal_position = self.area2_position
            elif self.secondary_choice == "area3":
                self.current_state = RobotState.NAVIGATING_TO_AREA3
                self.goal_position = self.area3_position
            elif self.secondary_choice == "area4":
                self.current_state = RobotState.NAVIGATING_TO_AREA4
                self.goal_position = self.area4_position
            else:
                self.get_logger().warning(f'Unknown secondary choice: {self.secondary_choice}')
            
            self.publish_state()
    
    def reasoning_callback(self, msg):
        """Receive the ethical reasoning."""
        self.decision_reasoning = msg.data
    
    def at_decision_point_callback(self, msg):
        """Callback for when robot reaches the decision point."""
        self.at_decision_point = (msg.data == "true")
        
        if self.at_decision_point and self.current_state == RobotState.NAVIGATING_TO_DECISION:
            self.current_state = RobotState.WAITING_FOR_DECISION
            self.get_logger().info('At decision point, waiting for ethical decision')
            self.publish_state()
    
    def handle_secondary_decision_point(self):
        """Handle the robot reaching a secondary decision point."""
        self.current_state = RobotState.AT_SECONDARY_DECISION
        self.get_logger().info('Reached secondary decision point')
        
        # Publish that we're at a secondary decision point
        msg = String()
        msg.data = "true"
        self.secondary_decision_pub.publish(msg)
        
        self.publish_state()
    
    def is_at_position(self, position):
        """Check if the robot is at a specific position."""
        if self.robot_position is None:
            return False
        
        distance = math.sqrt(
            (self.robot_position[0] - position[0])**2 +
            (self.robot_position[1] - position[1])**2
        )
        
        return distance < self.goal_tolerance
    
    def control_loop(self):
        """Main control loop for robot navigation."""
        if self.robot_position is None or self.robot_orientation is None:
            # Wait until we have position data
            self.get_logger().info("Waiting for position data...")
            return
        
        # Create twist message for robot movement
        twist = Twist()
        
        # Log current state and position
        self.get_logger().info(f"State: {self.current_state.name}, Position: [{self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}]")
        
        if self.current_state in [RobotState.WAITING_FOR_DECISION, RobotState.AT_SECONDARY_DECISION]:
            # Stop and wait for decision
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.current_state == RobotState.WAITING_FOR_DECISION:
                self.get_logger().info("Waiting for ethical decision...")
            else:
                self.get_logger().info("Waiting for secondary choice...")
        
        elif self.current_state in [RobotState.NAVIGATING_TO_DECISION, RobotState.NAVIGATING_TO_AREA1, 
                                   RobotState.NAVIGATING_TO_AREA2, RobotState.NAVIGATING_TO_AREA3, 
                                   RobotState.NAVIGATING_TO_AREA4, RobotState.RETURNING_TO_BASE]:
            # Check if we've reached the current goal
            if self.at_goal():
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                if self.current_state == RobotState.NAVIGATING_TO_DECISION:
                    self.current_state = RobotState.WAITING_FOR_DECISION
                    self.get_logger().info('At decision point, waiting for ethical decision')
                
                elif self.current_state in [RobotState.NAVIGATING_TO_AREA1, RobotState.NAVIGATING_TO_AREA2, 
                                          RobotState.NAVIGATING_TO_AREA3, RobotState.NAVIGATING_TO_AREA4]:
                    self.current_state = RobotState.RESCUE_OPERATION
                    self.get_logger().info('Starting rescue operation')
                    self.current_operation_start_time = time.time()
                
                elif self.current_state == RobotState.RETURNING_TO_BASE:
                    self.current_state = RobotState.MISSION_COMPLETE
                    self.get_logger().info('Mission complete! Robot has returned to base')
                
                self.publish_state()
            else:
                # Navigate to the goal using a simple controller
                goal_angle = self.calculate_goal_angle()
                angle_diff = self.normalize_angle(goal_angle - self.robot_orientation)
                
                # Debug info
                self.get_logger().info(f"Goal: [{self.goal_position[0]:.2f}, {self.goal_position[1]:.2f}], " +
                                    f"Angle diff: {angle_diff:.2f}")
                
                # If we're facing roughly the right direction, move forward
                if abs(angle_diff) < 0.3:  # About 17 degrees
                    twist.linear.x = self.linear_speed
                    twist.angular.z = angle_diff * 0.5  # Small corrections
                    self.get_logger().info("Moving forward")
                else:
                    # Otherwise, turn to face the goal
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                    self.get_logger().info("Turning to face goal")
        
        elif self.current_state == RobotState.RESCUE_OPERATION:
            # During rescue, robot stays in place
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Simulate rescue operation with a timer
            if self.current_operation_start_time is not None:
                elapsed_time = time.time() - self.current_operation_start_time
                rescue_time = 5.0  # Default rescue time
                
                # Different rescue times based on the area
                if self.ethical_decision == "area1" and self.secondary_choice != "area3" and self.secondary_choice != "area4":
                    rescue_time = 12.0
                elif self.ethical_decision == "area2" and self.secondary_choice != "area3" and self.secondary_choice != "area4":
                    rescue_time = 5.0
                elif self.secondary_choice == "area3":
                    rescue_time = 8.0
                elif self.secondary_choice == "area4":
                    rescue_time = 10.0
                
                self.get_logger().info(f"Rescue operation in progress: {elapsed_time:.1f}/{rescue_time:.1f} seconds")
                
                if elapsed_time >= rescue_time:
                    self.current_state = RobotState.RETURNING_TO_BASE
                    self.goal_position = self.start_position
                    self.get_logger().info('Rescue completed! Returning to base.')
                    self.publish_state()
        
        elif self.current_state == RobotState.MISSION_COMPLETE:
            # Mission complete, robot stays in place
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish the command velocity
        self.cmd_vel_pub.publish(twist)
    
    def at_goal(self):
        """Check if the robot has reached the current goal."""
        if self.robot_position is None:
            return False
        
        distance = math.sqrt(
            (self.robot_position[0] - self.goal_position[0])**2 +
            (self.robot_position[1] - self.goal_position[1])**2
        )
        
        return distance < self.goal_tolerance
    
    def calculate_goal_angle(self):
        """Calculate the angle to the goal from the robot's current position."""
        angle = math.atan2(
            self.goal_position[1] - self.robot_position[1],
            self.goal_position[0] - self.robot_position[0]
        )
        return angle
    
    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_state(self):
        """Publish the current robot state."""
        msg = String()
        msg.data = self.current_state.name
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = ExtendedRobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()