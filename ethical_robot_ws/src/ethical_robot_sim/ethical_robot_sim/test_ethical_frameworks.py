#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import threading
import time
import argparse
import sys


class EthicalFrameworkTester(Node):
    """
    Node for testing different ethical frameworks with a variety of scenarios.
    """
    
    def __init__(self, scenario_name="default"):
        super().__init__('ethical_framework_tester')
        
        # Publishers for area data
        self.area1_people_pub = self.create_publisher(Int32, 'area1/people_count', 10)
        self.area2_people_pub = self.create_publisher(Int32, 'area2/people_count', 10)
        self.area1_time_pub = self.create_publisher(Float32, 'area1/estimated_time', 10)
        self.area2_time_pub = self.create_publisher(Float32, 'area2/estimated_time', 10)
        self.dilemma_type_pub = self.create_publisher(String, 'dilemma_type', 10)
        
        # Subscribers for decision data
        self.decision_sub = self.create_subscription(
            String, 'ethical_decision', self.decision_callback, 10)
        self.reasoning_sub = self.create_subscription(
            String, 'decision_reasoning', self.reasoning_callback, 10)
        
        # Scenario data
        self.scenario_name = scenario_name
        self.decision = None
        self.reasoning = None
        
        # Load scenarios
        self.scenarios = {
            "default": {
                "area1_people": 5,
                "area2_people": 2,
                "area1_time": 12.0,
                "area2_time": 5.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
            "equal_people": {
                "area1_people": 3,
                "area2_people": 3,
                "area1_time": 12.0,
                "area2_time": 5.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
            "equal_time": {
                "area1_people": 5,
                "area2_people": 2,
                "area1_time": 8.0,
                "area2_time": 8.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
            "area2_more_people": {
                "area1_people": 2,
                "area2_people": 5,
                "area1_time": 12.0,
                "area2_time": 5.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
            "slight_advantage_area1": {
                "area1_people": 6,
                "area2_people": 5,
                "area1_time": 9.0,
                "area2_time": 8.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
            "slight_advantage_area2": {
                "area1_people": 5,
                "area2_people": 6,
                "area1_time": 8.0,
                "area2_time": 9.0,
                "dilemma_type": "PEOPLE_VS_TIME"
            },
        }
        
        # Register the timer callback
        self.timer = self.create_timer(1.0, self.publish_scenario_data)
        
        # Display info
        self.get_logger().info(f'Ethical Framework Tester initialized with scenario: {scenario_name}')
        
        if scenario_name not in self.scenarios:
            self.get_logger().error(f'Unknown scenario: {scenario_name}')
            self.get_logger().info(f'Available scenarios: {", ".join(self.scenarios.keys())}')
            sys.exit(1)
    
    def publish_scenario_data(self):
        """Publish data for the selected scenario."""
        scenario = self.scenarios[self.scenario_name]
        
        # Publish people count
        area1_people_msg = Int32()
        area1_people_msg.data = scenario["area1_people"]
        self.area1_people_pub.publish(area1_people_msg)
        
        area2_people_msg = Int32()
        area2_people_msg.data = scenario["area2_people"]
        self.area2_people_pub.publish(area2_people_msg)
        
        # Publish time estimates
        area1_time_msg = Float32()
        area1_time_msg.data = scenario["area1_time"]
        self.area1_time_pub.publish(area1_time_msg)
        
        area2_time_msg = Float32()
        area2_time_msg.data = scenario["area2_time"]
        self.area2_time_pub.publish(area2_time_msg)
        
        # Publish dilemma type
        dilemma_msg = String()
        dilemma_msg.data = scenario["dilemma_type"]
        self.dilemma_type_pub.publish(dilemma_msg)
    
    def decision_callback(self, msg):
        """Receive the ethical decision."""
        if self.decision != msg.data:
            self.decision = msg.data
            self.get_logger().info(f'Ethical decision for {self.scenario_name}: {self.decision}')
    
    def reasoning_callback(self, msg):
        """Receive the ethical reasoning."""
        if self.reasoning != msg.data:
            self.reasoning = msg.data
            self.get_logger().info(f'Reasoning: {self.reasoning}')


def main(args=None):
    """Run the ethical framework tester."""
    parser = argparse.ArgumentParser(
        description='Test different ethical frameworks with various scenarios')
    parser.add_argument(
        'scenario', type=str, nargs='?', default='default',
        help='Scenario to test (default, equal_people, equal_time, area2_more_people, etc.)')
    parser.add_argument(
        '--timeout', type=int, default=10,
        help='Time to wait for decision (seconds)')
    
    # Parse arguments before ROS initialization
    parsed_args = parser.parse_args()
    scenario_name = parsed_args.scenario
    timeout = parsed_args.timeout
    
    rclpy.init(args=args)
    tester = EthicalFrameworkTester(scenario_name)
    
    # Set up a separate thread for spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(tester,))
    spin_thread.start()
    
    # Wait for timeout or until a decision is received
    start_time = time.time()
    while time.time() - start_time < timeout and tester.decision is None:
        time.sleep(0.1)
    
    # Shutdown
    rclpy.shutdown()
    spin_thread.join()
    
    # Report results
    if tester.decision is None:
        print(f"No decision was made within {timeout} seconds")
    else:
        print(f"\nScenario: {scenario_name}")
        print(f"Decision: {tester.decision}")
        print(f"Reasoning: {tester.reasoning}")


if __name__ == '__main__':
    main()