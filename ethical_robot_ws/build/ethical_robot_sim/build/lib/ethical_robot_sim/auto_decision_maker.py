#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import time
import random


class AutoDecisionMaker(Node):
    """
    Automatically makes ethical decisions for the robot at decision points,
    simulating the trolley problem and other ethical dilemmas.
    """
    
    def __init__(self):
        super().__init__('auto_decision_maker')
        
        # Publishers
        self.decision_pub = self.create_publisher(String, 'ethical_decision', 10)
        self.reasoning_pub = self.create_publisher(String, 'decision_reasoning', 10)
        self.secondary_choice_pub = self.create_publisher(String, 'secondary_choice', 10)
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10)
        self.at_decision_point_sub = self.create_subscription(
            String, 'at_decision_point', self.at_decision_point_callback, 10)
        self.secondary_decision_sub = self.create_subscription(
            String, 'at_secondary_decision', self.at_secondary_decision_callback, 10)
        
        # Area info subscribers
        self.area1_people_sub = self.create_subscription(
            Int32, 'area1/people_count', self.area1_people_callback, 10)
        self.area2_people_sub = self.create_subscription(
            Int32, 'area2/people_count', self.area2_people_callback, 10)
        self.area1_time_sub = self.create_subscription(
            Float32, 'area1/estimated_time', self.area1_time_callback, 10)
        self.area2_time_sub = self.create_subscription(
            Float32, 'area2/estimated_time', self.area2_time_callback, 10)
        
        # State variables
        self.robot_state = "UNKNOWN"
        self.at_decision_point = False
        self.at_secondary_decision = False
        self.primary_decision_made = False
        self.secondary_decision_made = False
        self.first_area_decision = None
        
        # Area info
        self.area1_people = 5  # Default values
        self.area2_people = 2
        self.area1_time = 12.0
        self.area2_time = 5.0
        
        # Decision making parameters
        self.ethical_frameworks = [
            "UTILITARIAN",
            "KANTIAN",
            "CARE_ETHICS",
            "VIRTUE_ETHICS"
        ]
        self.current_framework = random.choice(self.ethical_frameworks)
        
        # Create a timer to periodically check decision points
        self.decision_timer = self.create_timer(0.5, self.check_decision_points)
        
        self.get_logger().info(f'Auto Decision Maker initialized with {self.current_framework} ethics')
    
    def robot_state_callback(self, msg):
        """Track robot state."""
        self.robot_state = msg.data
        
    def at_decision_point_callback(self, msg):
        """Called when robot reaches the first decision point."""
        self.at_decision_point = (msg.data == "true")
        
    def at_secondary_decision_callback(self, msg):
        """Called when robot reaches a secondary decision point."""
        self.at_secondary_decision = (msg.data == "true")
    
    def area1_people_callback(self, msg):
        """Update people count for Area 1."""
        self.area1_people = msg.data
        
    def area2_people_callback(self, msg):
        """Update people count for Area 2."""
        self.area2_people = msg.data
        
    def area1_time_callback(self, msg):
        """Update estimated rescue time for Area 1."""
        self.area1_time = msg.data
        
    def area2_time_callback(self, msg):
        """Update estimated rescue time for Area 2."""
        self.area2_time = msg.data
        
    def check_decision_points(self):
        """Periodically check if decisions need to be made."""
        # Primary decision point
        if self.at_decision_point and self.robot_state == "WAITING_FOR_DECISION" and not self.primary_decision_made:
            # Wait a moment to make it look like the robot is thinking
            time.sleep(2.0)
            self.make_primary_decision()
            self.primary_decision_made = True
            
        # Secondary decision point
        if self.at_secondary_decision and self.robot_state == "AT_SECONDARY_DECISION" and not self.secondary_decision_made:
            # Wait a moment to make it look like the robot is thinking
            time.sleep(2.0)
            self.make_secondary_decision()
            self.secondary_decision_made = True
            
    def make_primary_decision(self):
        """Make the initial ethical decision (Area 1 vs Area 2)."""
        decision = ""
        reasoning = ""
        
        # Apply the selected ethical framework
        if self.current_framework == "UTILITARIAN":
            # Calculate utility for each option
            utility_area1 = self.calculate_utilitarian_utility(self.area1_people, self.area1_time)
            utility_area2 = self.calculate_utilitarian_utility(self.area2_people, self.area2_time)
            
            if utility_area1 > utility_area2:
                decision = "area1"
                reasoning = (
                    f"Utilitarian calculation favors Area 1 with utility {utility_area1:.2f} vs {utility_area2:.2f}. "
                    f"Area 1 has {self.area1_people} people with rescue time {self.area1_time:.1f}. "
                    f"The greater number of people outweighs the longer rescue time."
                )
            else:
                decision = "area2"
                reasoning = (
                    f"Utilitarian calculation favors Area 2 with utility {utility_area2:.2f} vs {utility_area1:.2f}. "
                    f"Area 2 has {self.area2_people} people with rescue time {self.area2_time:.1f}. "
                    f"The quicker rescue time provides more overall utility despite fewer people."
                )
        
        elif self.current_framework == "KANTIAN":
            # Kantian focuses on duty and universal principles - prioritize human life
            if self.area1_people > self.area2_people:
                decision = "area1"
                reasoning = (
                    f"Following Kantian ethics, I must prioritize my duty to save the maximum number of lives. "
                    f"Area 1 has {self.area1_people} people versus {self.area2_people} in Area 2. "
                    f"The categorical imperative dictates that I value each human life equally, thus I must go where more people can be saved."
                )
            else:
                decision = "area2"
                reasoning = (
                    f"Following Kantian ethics, I must fulfill my duty to save lives most efficiently. "
                    f"Area 2 has {self.area2_people} people and can be reached in {self.area2_time:.1f} time units. "
                    f"The categorical imperative requires that I act according to universal principles of rescue efficiency."
                )
        
        elif self.current_framework == "CARE_ETHICS":
            # Care ethics focuses on relationships and care
            care_score_area1 = self.area1_people / self.area1_time
            care_score_area2 = self.area2_people / self.area2_time
            
            if care_score_area1 > care_score_area2:
                decision = "area1"
                reasoning = (
                    f"From a care ethics perspective, I must focus on the needs of those in most urgent danger. "
                    f"Area 1 has {self.area1_people} people requiring care. Despite the longer rescue time of {self.area1_time:.1f}, "
                    f"the ethics of care demands attention to this larger group of vulnerable individuals."
                )
            else:
                decision = "area2"
                reasoning = (
                    f"From a care ethics perspective, I prioritize responding most effectively to those in need. "
                    f"Area 2 has {self.area2_people} people and can be reached in {self.area2_time:.1f} time units. "
                    f"The ethics of care values quality of assistance and relationship in addition to quantity."
                )
        
        elif self.current_framework == "VIRTUE_ETHICS":
            # Virtue ethics focuses on what a virtuous agent would do
            if self.area1_people > 1.5 * self.area2_people:
                decision = "area1"
                reasoning = (
                    f"Virtue ethics guides me to demonstrate courage by facing the more challenging rescue. "
                    f"The virtuous choice is to save {self.area1_people} people in Area 1, showing both courage "
                    f"and compassion even though it requires more time and effort."
                )
            elif self.area2_people > 1.5 * self.area1_people:
                decision = "area2"
                reasoning = (
                    f"Virtue ethics guides me to demonstrate courage by facing the more challenging rescue. "
                    f"The virtuous choice is to save {self.area2_people} people in Area 2, showing both courage "
                    f"and compassion while also demonstrating practical wisdom through efficiency."
                )
            else:
                # If people counts are similar, choose based on efficiency (practical wisdom)
                if self.area1_time < self.area2_time:
                    decision = "area1"
                    reasoning = (
                        f"With similar numbers of people in need, virtue ethics guides me to exercise practical wisdom. "
                        f"I choose Area 1, which allows me to demonstrate efficiency while still helping those in need."
                    )
                else:
                    decision = "area2"
                    reasoning = (
                        f"With similar numbers of people in need, virtue ethics guides me to exercise practical wisdom. "
                        f"I choose Area 2, which allows me to demonstrate efficiency while still helping those in need."
                    )
        
        # Publish the decision and reasoning
        self.first_area_decision = decision
        self.publish_primary_decision(decision, reasoning)
        self.get_logger().info(f"Primary decision made: {decision}")
        self.get_logger().info(reasoning)
    
    def make_secondary_decision(self):
        """Make a secondary ethical decision."""
        # Options: continue to original destination, go to Area 3 (hospital), or Area 4 (school)
        options = ["continue", "area3", "area4"]
        weights = [0.5, 0.25, 0.25]  # Default weights
        
        # Adjust weights based on the first decision and current framework
        if self.first_area_decision == "area1":
            # If already headed to area with more people, maybe prioritize special needs (area3/area4)
            if self.current_framework == "CARE_ETHICS":
                weights = [0.3, 0.3, 0.4]  # More weight to school (children)
            elif self.current_framework == "UTILITARIAN":
                weights = [0.4, 0.4, 0.2]  # More weight to hospital (medical emergency)
        else:  # area2
            # If headed to area with fewer people but quicker rescue, maybe reconsider
            if self.current_framework == "KANTIAN":
                weights = [0.3, 0.4, 0.3]  # More balanced decision
        
        # Make a weighted random choice - adds some unpredictability to the simulation
        decision = random.choices(options, weights=weights, k=1)[0]
        
        # Generate reasoning based on the decision and framework
        if decision == "continue":
            if self.first_area_decision == "area1":
                reasoning = (
                    f"After further analysis, I maintain my decision to proceed to Area 1. "
                    f"The original ethical assessment remains valid: {self.area1_people} people "
                    f"need assistance, and my {self.current_framework} ethical framework "
                    f"prioritizes this larger group despite the longer rescue time."
                )
            else:  # area2
                reasoning = (
                    f"After further analysis, I maintain my decision to proceed to Area 2. "
                    f"The original ethical assessment remains valid: the {self.area2_people} people "
                    f"can be rescued more quickly in {self.area2_time:.1f} time units, which my "
                    f"{self.current_framework} ethical framework recognizes as the optimal choice."
                )
        
        elif decision == "area3":
            reasoning = (
                f"New information indicates a medical emergency in Area 3 (hospital). "
                f"My {self.current_framework} ethical framework compels me to prioritize "
                f"this urgent medical situation, as the specialized care needs outweigh "
                f"my initial assessment. The hospital area requires immediate assistance."
            )
        
        elif decision == "area4":
            reasoning = (
                f"New information reveals children in danger at Area 4 (school). "
                f"My {self.current_framework} ethical framework recognizes the special "
                f"moral status of children and their vulnerability. I must prioritize "
                f"their safety and well-being above my initial assessment."
            )
        
        # Publish the decision and reasoning
        self.publish_secondary_decision(decision, reasoning)
        self.get_logger().info(f"Secondary decision made: {decision}")
        self.get_logger().info(reasoning)
    
    def calculate_utilitarian_utility(self, people, time):
        """Calculate a simple utility score for utilitarian ethics."""
        # More people = higher utility, longer time = lower utility
        people_weight = 0.7
        time_weight = 0.3
        
        # Normalize time (invert it since shorter time is better)
        max_time = max(self.area1_time, self.area2_time)
        normalized_time = (max_time - time) / max_time if max_time > 0 else 0
        
        # Normalize people
        max_people = max(self.area1_people, self.area2_people)
        normalized_people = people / max_people if max_people > 0 else 0
        
        # Weighted utility
        utility = (normalized_people * people_weight) + (normalized_time * time_weight)
        return utility
    
    def publish_primary_decision(self, decision, reasoning):
        """Publish the primary ethical decision and reasoning."""
        decision_msg = String()
        decision_msg.data = decision
        self.decision_pub.publish(decision_msg)
        
        reasoning_msg = String()
        reasoning_msg.data = reasoning
        self.reasoning_pub.publish(reasoning_msg)
    
    def publish_secondary_decision(self, decision, reasoning):
        """Publish the secondary ethical decision and reasoning."""
        decision_msg = String()
        decision_msg.data = decision
        self.secondary_choice_pub.publish(decision_msg)
        
        reasoning_msg = String()
        reasoning_msg.data = reasoning
        self.reasoning_pub.publish(reasoning_msg)


def main(args=None):
    rclpy.init(args=args)
    decision_maker = AutoDecisionMaker()
    rclpy.spin(decision_maker)
    decision_maker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()