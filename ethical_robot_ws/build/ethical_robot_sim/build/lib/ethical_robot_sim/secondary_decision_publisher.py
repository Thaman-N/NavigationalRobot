#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class SecondaryDecisionPublisher(Node):
    """Node to publish secondary decisions when the robot reaches a secondary decision point."""
    
    def __init__(self):
        super().__init__('secondary_decision_publisher')
        
        # Create publisher for secondary decisions
        self.secondary_choice_pub = self.create_publisher(
            String, 'secondary_choice', 10)
        
        # Subscribe to know when robot is at a secondary decision point
        self.at_secondary_decision_sub = self.create_subscription(
            String, 'at_secondary_decision', self.at_secondary_decision_callback, 10)
        
        # Subscribe to robot state
        self.robot_state_sub = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10)
        
        self.at_secondary_decision = False
        self.robot_state = ""
        
        self.get_logger().info('Secondary Decision Publisher initialized')
        
    def at_secondary_decision_callback(self, msg):
        """Callback for when robot reaches a secondary decision point."""
        self.at_secondary_decision = (msg.data == "true")
        if self.at_secondary_decision:
            self.get_logger().info('Robot is at a secondary decision point')
            self.get_logger().info('Options: "continue", "area3", or "area4"')
    
    def robot_state_callback(self, msg):
        """Track robot state."""
        self.robot_state = msg.data
    
    def publish_decision(self, decision):
        """Publish a secondary decision."""
        msg = String()
        msg.data = decision
        self.secondary_choice_pub.publish(msg)
        self.get_logger().info(f'Published secondary decision: {decision}')


def main(args=None):
    """Run the secondary decision publisher."""
    rclpy.init(args=args)
    publisher = SecondaryDecisionPublisher()
    
    if len(sys.argv) > 1:
        # If command line argument is provided, publish it and exit
        decision = sys.argv[1]
        time.sleep(1)  # Give time for subscribers to connect
        publisher.publish_decision(decision)
        time.sleep(1)  # Give time for the message to be received
        rclpy.shutdown()
        return
    
    # Otherwise, spin to keep checking for secondary decision points
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()