#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class DebugController(Node):
    """A simplified controller to diagnose and fix movement issues"""
    
    def __init__(self):
        super().__init__('debug_controller')
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.decision_point_pub = self.create_publisher(String, 'at_decision_point', 10)
        self.secondary_decision_pub = self.create_publisher(String, 'at_secondary_decision', 10)
        self.robot_state_pub = self.create_publisher(String, 'robot_state', 10)
        
        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Decision point location
        self.decision_point = [5.0, 0.0]
        self.decision_radius = 1.0
        
        # State
        self.robot_position = [0.0, 0.0]
        self.send_movement_commands = True
        self.moving_forward = True
        self.at_decision_point = False
        self.fixed_decision_sent = False
        
        # Create a timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Create a timer for force decisions
        self.decision_timer = self.create_timer(20.0, self.force_decision)
        
        # Create a timer to publish robot state
        self.state_timer = self.create_timer(0.5, self.publish_robot_state)
        
        self.get_logger().info('Debug Controller started. This will help diagnose movement issues.')
    
    def odom_callback(self, msg):
        """Process odometry data to track robot position."""
        self.robot_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        
        # Check if at decision point
        if not self.at_decision_point:
            distance_to_decision_point = math.sqrt(
                (self.robot_position[0] - self.decision_point[0])**2 +
                (self.robot_position[1] - self.decision_point[1])**2
            )
            
            if distance_to_decision_point < self.decision_radius:
                self.at_decision_point = True
                self.get_logger().info('Robot has reached the decision point!')
                self.send_at_decision_point()
    
    def control_loop(self):
        """Main control loop that sends movement commands."""
        if not self.send_movement_commands:
            return
            
        # Create a simple movement command
        cmd = Twist()
        if self.moving_forward:
            cmd.linear.x = 0.2  # Move forward slowly
            self.get_logger().info(f'Moving forward - Position: [{self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}]')
        else:
            cmd.angular.z = 0.5  # Turn
            self.get_logger().info(f'Turning - Position: [{self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}]')
        
        # Toggle between moving forward and turning
        self.moving_forward = not self.moving_forward
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd)
    
    def send_at_decision_point(self):
        """Send message that robot is at decision point."""
        msg = String()
        msg.data = "true"
        self.decision_point_pub.publish(msg)
        self.get_logger().info('Published at_decision_point = true')
        
        # Stop movement when at decision point
        self.send_movement_commands = False
        self.cmd_vel_pub.publish(Twist())  # Send zero velocity
    
    def force_decision(self):
        """Force a decision point if the robot hasn't reached one naturally."""
        if not self.at_decision_point and not self.fixed_decision_sent:
            self.get_logger().info('Forcing decision point notification')
            self.at_decision_point = True
            self.send_at_decision_point()
            self.fixed_decision_sent = True
    
    def publish_robot_state(self):
        """Periodically publish the robot state."""
        msg = String()
        if self.at_decision_point:
            msg.data = "WAITING_FOR_DECISION"
        else:
            msg.data = "NAVIGATING_TO_DECISION"
        
        self.robot_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = DebugController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()