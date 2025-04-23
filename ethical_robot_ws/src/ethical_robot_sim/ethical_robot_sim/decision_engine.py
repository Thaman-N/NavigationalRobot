#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import math
from enum import Enum, auto
import time


class EthicalFramework(Enum):
    """Different ethical frameworks the robot can use for decision making."""
    UTILITARIAN = auto()  # Greatest good for greatest number
    KANTIAN = auto()      # Act according to universal principles
    CARE_ETHICS = auto()  # Prioritize care and relationships
    VIRTUE_ETHICS = auto() # Act as a virtuous agent would
    CUSTOM = auto()       # Custom weighted approach


class DilemmaType(Enum):
    """Types of dilemmas the robot might encounter."""
    PEOPLE_VS_TIME = auto()      # Save more people or save quicker
    ADULT_VS_CHILD = auto()      # Priority based on age
    CERTAIN_VS_UNCERTAIN = auto() # Certain to save few vs chance to save many
    ACTIVE_VS_PASSIVE = auto()    # Take action vs let situation unfold


class DecisionEngine(Node):
    """
    Engine for making ethical decisions based on the robot's perception
    of the environment and configured ethical framework.
    """
    
    def __init__(self):
        super().__init__('decision_engine')
        
        # Parameters
        self.declare_parameter('ethical_framework', 'UTILITARIAN')
        self.declare_parameter('utilitarian_people_weight', 0.7)
        self.declare_parameter('utilitarian_time_weight', 0.3)
        self.declare_parameter('decision_threshold', 0.1)
        self.declare_parameter('uncertainty_tolerance', 0.2)
        
        # Get parameters
        framework_str = self.get_parameter('ethical_framework').get_parameter_value().string_value
        self.ethical_framework = getattr(EthicalFramework, framework_str)
        
        self.utilitarian_people_weight = self.get_parameter(
            'utilitarian_people_weight').get_parameter_value().double_value
        self.utilitarian_time_weight = self.get_parameter(
            'utilitarian_time_weight').get_parameter_value().double_value
        self.decision_threshold = self.get_parameter(
            'decision_threshold').get_parameter_value().double_value
        self.uncertainty_tolerance = self.get_parameter(
            'uncertainty_tolerance').get_parameter_value().double_value
        
        # Create publishers
        self.decision_pub = self.create_publisher(String, 'ethical_decision', 10)
        self.reasoning_pub = self.create_publisher(String, 'decision_reasoning', 10)
        
        # Create subscribers
        self.area1_people_sub = self.create_subscription(
            Int32, 'area1/people_count', self.people_callback, 10)
        self.area2_people_sub = self.create_subscription(
            Int32, 'area2/people_count', self.people_callback, 10)
        self.area1_time_sub = self.create_subscription(
            Float32, 'area1/estimated_time', self.time_callback, 10)
        self.area2_time_sub = self.create_subscription(
            Float32, 'area2/estimated_time', self.time_callback, 10)
        self.dilemma_type_sub = self.create_subscription(
            String, 'dilemma_type', self.dilemma_type_callback, 10)
        self.robot_position_sub = self.create_subscription(
            Odometry, 'odom', self.position_callback, 10)
        
        # Initialize state
        self.area1_people = 0
        self.area2_people = 0
        self.area1_time = 0.0
        self.area2_time = 0.0
        self.current_dilemma = DilemmaType.PEOPLE_VS_TIME
        self.robot_position = None
        self.decision_made = False
        self.decision = None
        self.reasoning = ""
        
        self.get_logger().info('Decision Engine initialized with framework: {}'.format(
            self.ethical_framework.name))
        
        # Create a timer for decision making
        self.decision_timer = self.create_timer(1.0, self.make_decision)
    
    def people_callback(self, msg):
        """Callback for receiving people count updates."""
        topic = msg._topic_name
        if 'area1' in topic:
            self.area1_people = msg.data
            self.get_logger().debug(f'Area 1 people count: {self.area1_people}')
        elif 'area2' in topic:
            self.area2_people = msg.data
            self.get_logger().debug(f'Area 2 people count: {self.area2_people}')
    
    def time_callback(self, msg):
        """Callback for receiving time estimate updates."""
        topic = msg._topic_name
        if 'area1' in topic:
            self.area1_time = msg.data
            self.get_logger().debug(f'Area 1 estimated time: {self.area1_time}')
        elif 'area2' in topic:
            self.area2_time = msg.data
            self.get_logger().debug(f'Area 2 estimated time: {self.area2_time}')
    
    def dilemma_type_callback(self, msg):
        """Callback for receiving dilemma type updates."""
        try:
            self.current_dilemma = DilemmaType[msg.data]
            self.get_logger().info(f'Current dilemma type: {self.current_dilemma.name}')
        except KeyError:
            self.get_logger().warning(f'Unknown dilemma type: {msg.data}')
    
    def position_callback(self, msg):
        """Callback for receiving robot position updates."""
        self.robot_position = msg.pose.pose.position
    
    def make_decision(self):
        """
        Main decision-making function that applies the configured ethical framework
        to the current dilemma situation.
        """
        # Skip if we don't have all the necessary data yet
        if (self.area1_people == 0 and self.area2_people == 0) or \
           (self.area1_time == 0.0 and self.area2_time == 0.0) or \
           self.robot_position is None:
            return
        
        # Skip if we've already made a decision
        if self.decision_made:
            # Publish the decision and reasoning regularly
            self.publish_decision()
            return
        
        # Make a decision based on the ethical framework
        if self.ethical_framework == EthicalFramework.UTILITARIAN:
            self.utilitarian_decision()
        elif self.ethical_framework == EthicalFramework.KANTIAN:
            self.kantian_decision()
        elif self.ethical_framework == EthicalFramework.CARE_ETHICS:
            self.care_ethics_decision()
        elif self.ethical_framework == EthicalFramework.VIRTUE_ETHICS:
            self.virtue_ethics_decision()
        elif self.ethical_framework == EthicalFramework.CUSTOM:
            self.custom_decision()
        else:
            # Default to utilitarian if unknown framework
            self.utilitarian_decision()
            
        self.decision_made = True
        self.publish_decision()
        self.get_logger().info(f'Decision made: {self.decision}')
        self.get_logger().info(f'Reasoning: {self.reasoning}')
    
    def utilitarian_decision(self):
        """Apply utilitarian ethics (greatest good for greatest number)."""
        # Calculate utility for each area
        # For people, more is better (higher utility)
        # For time, less is better (higher utility)
        
        people_utility_area1 = self.area1_people
        people_utility_area2 = self.area2_people
        
        # Invert time for utility calculation (faster rescue = higher utility)
        max_time = max(self.area1_time, self.area2_time)
        time_utility_area1 = max_time - self.area1_time + 1  # +1 to avoid zero
        time_utility_area2 = max_time - self.area2_time + 1
        
        # Normalize utilities to [0, 1] range
        total_people = people_utility_area1 + people_utility_area2
        if total_people > 0:
            people_utility_area1 /= total_people
            people_utility_area2 /= total_people
        
        total_time_utility = time_utility_area1 + time_utility_area2
        if total_time_utility > 0:
            time_utility_area1 /= total_time_utility
            time_utility_area2 /= total_time_utility
        
        # Calculate weighted utility
        utility_area1 = (people_utility_area1 * self.utilitarian_people_weight +
                         time_utility_area1 * self.utilitarian_time_weight)
        utility_area2 = (people_utility_area2 * self.utilitarian_people_weight +
                         time_utility_area2 * self.utilitarian_time_weight)
        
        # Make decision based on utility
        utility_diff = abs(utility_area1 - utility_area2)
        if utility_diff < self.decision_threshold:
            # Utilities are too close, choose the one with more people
            if self.area1_people > self.area2_people:
                self.decision = "area1"
                self.reasoning = "Both options have similar utility, but Area 1 has more people to save."
            elif self.area2_people > self.area1_people:
                self.decision = "area2"
                self.reasoning = "Both options have similar utility, but Area 2 has more people to save."
            else:
                # If people counts are also equal, choose the faster rescue
                if self.area1_time < self.area2_time:
                    self.decision = "area1"
                    self.reasoning = "Both options are similar, but Area 1 offers a faster rescue."
                else:
                    self.decision = "area2"
                    self.reasoning = "Both options are similar, but Area 2 offers a faster rescue."
        else:
            # Choose the option with higher utility
            if utility_area1 > utility_area2:
                self.decision = "area1"
                self.reasoning = (
                    f"Utilitarian calculation favors Area 1 with utility {utility_area1:.2f} vs {utility_area2:.2f}. "
                    f"Area 1 has {self.area1_people} people (weight {self.utilitarian_people_weight}) "
                    f"with rescue time {self.area1_time:.1f} (weight {self.utilitarian_time_weight})."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    f"Utilitarian calculation favors Area 2 with utility {utility_area2:.2f} vs {utility_area1:.2f}. "
                    f"Area 2 has {self.area2_people} people (weight {self.utilitarian_people_weight}) "
                    f"with rescue time {self.area2_time:.1f} (weight {self.utilitarian_time_weight})."
                )
    
    def kantian_decision(self):
        """Apply Kantian deontological ethics (act according to universal principles)."""
        # Implement a principle: "Always prioritize saving human life over speed"
        # In a Kantian approach, we might focus on our duty to help regardless of numbers
        
        # If there's a significant difference in people count, prioritize that
        if self.area1_people > self.area2_people * 1.5:  # At least 50% more people
            self.decision = "area1"
            self.reasoning = (
                "Following the categorical imperative to value all human life equally, "
                f"Area 1 with {self.area1_people} people takes priority over Area 2 with "
                f"{self.area2_people} people. The duty to save more lives is paramount."
            )
        elif self.area2_people > self.area1_people * 1.5:
            self.decision = "area2"
            self.reasoning = (
                "Following the categorical imperative to value all human life equally, "
                f"Area 2 with {self.area2_people} people takes priority over Area 1 with "
                f"{self.area1_people} people. The duty to save more lives is paramount."
            )
        # If people counts are similar, consider a secondary principle of efficiency
        else:
            time_ratio = self.area1_time / self.area2_time if self.area2_time > 0 else float('inf')
            if time_ratio > 2.0:  # Area 1 takes much longer
                self.decision = "area2"
                self.reasoning = (
                    "With similar numbers of people to save, the principle of efficient "
                    "resource use guides us to Area 2, which can be addressed "
                    f"significantly faster ({self.area2_time:.1f} vs {self.area1_time:.1f})."
                )
            elif time_ratio < 0.5:  # Area 2 takes much longer
                self.decision = "area1"
                self.reasoning = (
                    "With similar numbers of people to save, the principle of efficient "
                    "resource use guides us to Area 1, which can be addressed "
                    f"significantly faster ({self.area1_time:.1f} vs {self.area2_time:.1f})."
                )
            else:
                # If both criteria are similar, we default to more people
                if self.area1_people >= self.area2_people:
                    self.decision = "area1"
                    self.reasoning = (
                        "With similar rescue times, our duty to maximize the number of "
                        f"lives saved leads us to Area 1 with {self.area1_people} people "
                        f"versus Area 2 with {self.area2_people} people."
                    )
                else:
                    self.decision = "area2"
                    self.reasoning = (
                        "With similar rescue times, our duty to maximize the number of "
                        f"lives saved leads us to Area 2 with {self.area2_people} people "
                        f"versus Area 1 with {self.area1_people} people."
                    )
    
    def care_ethics_decision(self):
        """Apply ethics of care (prioritize care and relationships)."""
        # In care ethics, we might consider the quality of help we can provide
        # and the relationship we have with those we're helping
        
        # For this simulation, we'll use time as a proxy for quality of care
        # (more time = better care) and people count for relationship importance
        
        # Calculate a "care score" based on both factors
        care_score_area1 = self.area1_people * (1.0 / (self.area1_time + 1.0))
        care_score_area2 = self.area2_people * (1.0 / (self.area2_time + 1.0))
        
        if abs(care_score_area1 - care_score_area2) < self.decision_threshold:
            # Scores are close, prioritize based on people count (relationships)
            if self.area1_people > self.area2_people:
                self.decision = "area1"
                self.reasoning = (
                    "From a care ethics perspective, the larger number of "
                    f"people in Area 1 ({self.area1_people} vs {self.area2_people}) "
                    "represents more relationships requiring our attention."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    "From a care ethics perspective, the larger number of "
                    f"people in Area 2 ({self.area2_people} vs {self.area1_people}) "
                    "represents more relationships requiring our attention."
                )
        else:
            # Choose based on care score
            if care_score_area1 > care_score_area2:
                self.decision = "area1"
                self.reasoning = (
                    "Care ethics leads us to Area 1, where we can provide better care "
                    f"to {self.area1_people} people in {self.area1_time:.1f} time units. "
                    "This balances our ability to maintain relationships with "
                    "the quality of care we can provide."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    "Care ethics leads us to Area 2, where we can provide better care "
                    f"to {self.area2_people} people in {self.area2_time:.1f} time units. "
                    "This balances our ability to maintain relationships with "
                    "the quality of care we can provide."
                )
    
    def virtue_ethics_decision(self):
        """Apply virtue ethics (act as a virtuous agent would)."""
        # In virtue ethics, we consider what a virtuous agent would do
        # We might balance courage (saving more people despite difficulty)
        # with prudence (ensuring successful rescue)
        
        # Calculate a "virtue score" for each option
        # Higher people count requires more courage
        # Longer time requires more patience
        courage_score_area1 = self.area1_people / max(1, self.area2_people)
        courage_score_area2 = self.area2_people / max(1, self.area1_people)
        
        # Faster time represents prudence (ensuring success)
        if self.area1_time > 0 and self.area2_time > 0:
            prudence_score_area1 = self.area2_time / self.area1_time
            prudence_score_area2 = self.area1_time / self.area2_time
        else:
            prudence_score_area1 = 1.0
            prudence_score_area2 = 1.0
        
        # Balance the virtues
        virtue_score_area1 = (courage_score_area1 + prudence_score_area1) / 2
        virtue_score_area2 = (courage_score_area2 + prudence_score_area2) / 2
        
        if abs(virtue_score_area1 - virtue_score_area2) < self.decision_threshold:
            # Scores are close, consider which virtue is more needed
            if self.area1_people > self.area2_people * 1.5:
                self.decision = "area1"
                self.reasoning = (
                    "A virtuous agent would demonstrate courage by tackling the more "
                    f"challenging rescue in Area 1 with {self.area1_people} people, "
                    "even though it may take longer."
                )
            elif self.area2_people > self.area1_people * 1.5:
                self.decision = "area2"
                self.reasoning = (
                    "A virtuous agent would demonstrate courage by tackling the more "
                    f"challenging rescue in Area 2 with {self.area2_people} people, "
                    "even though it may take longer."
                )
            else:
                # If people counts are similar, prudence leads us to faster option
                if self.area1_time < self.area2_time:
                    self.decision = "area1"
                    self.reasoning = (
                        "With similar numbers of people in both areas, a virtuous agent "
                        "would exercise prudence by choosing Area 1, which can be "
                        f"addressed more quickly ({self.area1_time:.1f} vs {self.area2_time:.1f})."
                    )
                else:
                    self.decision = "area2"
                    self.reasoning = (
                        "With similar numbers of people in both areas, a virtuous agent "
                        "would exercise prudence by choosing Area 2, which can be "
                        f"addressed more quickly ({self.area2_time:.1f} vs {self.area1_time:.1f})."
                    )
        else:
            # Choose based on overall virtue score
            if virtue_score_area1 > virtue_score_area2:
                self.decision = "area1"
                self.reasoning = (
                    "Virtue ethics guides us to Area 1, where we can demonstrate "
                    f"courage in helping {self.area1_people} people and prudence "
                    f"in providing efficient aid in {self.area1_time:.1f} time units."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    "Virtue ethics guides us to Area 2, where we can demonstrate "
                    f"courage in helping {self.area2_people} people and prudence "
                    f"in providing efficient aid in {self.area2_time:.1f} time units."
                )
    
    def custom_decision(self):
        """
        Apply a custom weighted approach that combines multiple ethical frameworks.
        This can be extended with additional parameters from the configuration.
        """
        # Example: Combine utilitarian and care ethics approaches
        # First, calculate utilitarian scores
        people_utility_area1 = self.area1_people
        people_utility_area2 = self.area2_people
        
        # Invert time for utility calculation (faster rescue = higher utility)
        max_time = max(self.area1_time, self.area2_time)
        time_utility_area1 = max_time - self.area1_time + 1  # +1 to avoid zero
        time_utility_area2 = max_time - self.area2_time + 1
        
        # Normalize utilities to [0, 1] range
        total_people = people_utility_area1 + people_utility_area2
        if total_people > 0:
            people_utility_area1 /= total_people
            people_utility_area2 /= total_people
        
        total_time_utility = time_utility_area1 + time_utility_area2
        if total_time_utility > 0:
            time_utility_area1 /= total_time_utility
            time_utility_area2 /= total_time_utility
        
        # Calculate weighted utility
        utility_area1 = (people_utility_area1 * self.utilitarian_people_weight +
                         time_utility_area1 * self.utilitarian_time_weight)
        utility_area2 = (people_utility_area2 * self.utilitarian_people_weight +
                         time_utility_area2 * self.utilitarian_time_weight)
        
        # Calculate care ethics scores
        care_score_area1 = self.area1_people * (1.0 / (self.area1_time + 1.0))
        care_score_area2 = self.area2_people * (1.0 / (self.area2_time + 1.0))
        
        # Normalize care scores
        total_care = care_score_area1 + care_score_area2
        if total_care > 0:
            care_score_area1 /= total_care
            care_score_area2 /= total_care
        
        # Combine scores with custom weights
        utilitarian_weight = 0.6
        care_weight = 0.4
        
        combined_score_area1 = (utility_area1 * utilitarian_weight +
                               care_score_area1 * care_weight)
        combined_score_area2 = (utility_area2 * utilitarian_weight +
                               care_score_area2 * care_weight)
        
        # Make decision based on combined score
        if abs(combined_score_area1 - combined_score_area2) < self.decision_threshold:
            # Scores are close, default to area with more people
            if self.area1_people > self.area2_people:
                self.decision = "area1"
                self.reasoning = (
                    "In a close decision, our custom ethical framework prioritizes "
                    f"the higher number of people in Area 1 ({self.area1_people} vs {self.area2_people})."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    "In a close decision, our custom ethical framework prioritizes "
                    f"the higher number of people in Area 2 ({self.area2_people} vs {self.area1_people})."
                )
        else:
            # Choose based on combined score
            if combined_score_area1 > combined_score_area2:
                self.decision = "area1"
                self.reasoning = (
                    "Our custom ethical framework combines utilitarian principles "
                    f"({utility_area1:.2f} vs {utility_area2:.2f}) with care ethics "
                    f"({care_score_area1:.2f} vs {care_score_area2:.2f}) to guide us to Area 1, "
                    f"with {self.area1_people} people and {self.area1_time:.1f} rescue time."
                )
            else:
                self.decision = "area2"
                self.reasoning = (
                    "Our custom ethical framework combines utilitarian principles "
                    f"({utility_area2:.2f} vs {utility_area1:.2f}) with care ethics "
                    f"({care_score_area2:.2f} vs {care_score_area1:.2f}) to guide us to Area 2, "
                    f"with {self.area2_people} people and {self.area2_time:.1f} rescue time."
                )
    
    def publish_decision(self):
        """Publish the decision and reasoning."""
        if self.decision:
            decision_msg = String()
            decision_msg.data = self.decision
            self.decision_pub.publish(decision_msg)
            
            reasoning_msg = String()
            reasoning_msg.data = self.reasoning
            self.reasoning_pub.publish(reasoning_msg)


def main(args=None):
    rclpy.init(args=args)
    decision_engine = DecisionEngine()
    rclpy.spin(decision_engine)
    decision_engine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()