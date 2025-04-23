#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches


class DecisionVisualizer(Node):
    """
    A node that visualizes ethical decisions and their factors in real-time.
    """
    
    def __init__(self):
        super().__init__('decision_visualizer')
        
        # Subscribe to relevant topics
        self.area1_people_sub = self.create_subscription(
            Int32, 'area1/people_count', self.area1_people_callback, 10)
        self.area2_people_sub = self.create_subscription(
            Int32, 'area2/people_count', self.area2_people_callback, 10)
        self.area1_time_sub = self.create_subscription(
            Float32, 'area1/estimated_time', self.area1_time_callback, 10)
        self.area2_time_sub = self.create_subscription(
            Float32, 'area2/estimated_time', self.area2_time_callback, 10)
        self.decision_sub = self.create_subscription(
            String, 'ethical_decision', self.decision_callback, 10)
        self.reasoning_sub = self.create_subscription(
            String, 'decision_reasoning', self.reasoning_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10)
        
        # Initialize data
        self.area1_people = 0
        self.area2_people = 0
        self.area1_time = 0.0
        self.area2_time = 0.0
        self.decision = None
        self.reasoning = "No decision yet"
        self.robot_state = "Unknown"
        
        # Visualization setup
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Ethical Decision Visualization', fontsize=16)
        
        # Setup subplots
        self.axs[0, 0].set_title('People Count')
        self.axs[0, 0].set_xlabel('Area')
        self.axs[0, 0].set_ylabel('Number of People')
        self.bar_people = self.axs[0, 0].bar(['Area 1', 'Area 2'], [0, 0], color=['blue', 'green'])
        self.axs[0, 0].set_ylim(0, 10)
        
        self.axs[0, 1].set_title('Rescue Time')
        self.axs[0, 1].set_xlabel('Area')
        self.axs[0, 1].set_ylabel('Time (seconds)')
        self.bar_time = self.axs[0, 1].bar(['Area 1', 'Area 2'], [0, 0], color=['blue', 'green'])
        self.axs[0, 1].set_ylim(0, 15)
        
        # Decision status plot (shows which area was chosen)
        self.axs[1, 0].set_title('Decision Status')
        self.axs[1, 0].set_xlim(-1, 1)
        self.axs[1, 0].set_ylim(-1, 1)
        self.axs[1, 0].set_aspect('equal')
        self.axs[1, 0].axis('off')
        
        # Create area indicators
        self.area1_patch = patches.Circle((-0.5, 0), 0.4, fc='blue', alpha=0.3)
        self.area2_patch = patches.Circle((0.5, 0), 0.4, fc='green', alpha=0.3)
        self.axs[1, 0].add_patch(self.area1_patch)
        self.axs[1, 0].add_patch(self.area2_patch)
        self.axs[1, 0].text(-0.5, 0, "Area 1", ha='center', va='center')
        self.axs[1, 0].text(0.5, 0, "Area 2", ha='center', va='center')
        
        # Decision reasoning text box
        self.axs[1, 1].set_title('Decision Reasoning')
        self.axs[1, 1].axis('off')
        self.reasoning_text = self.axs[1, 1].text(0.5, 0.5, "No decision yet",
                                                  ha='center', va='center',
                                                  wrap=True)
        
        # Status bar at the bottom
        self.status_text = self.fig.text(0.5, 0.01, f"Robot State: {self.robot_state}", 
                                         ha='center', va='center')
        
        # Create animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)
        
        self.get_logger().info('Decision Visualizer initialized')
    
    def area1_people_callback(self, msg):
        """Callback for Area 1 people count."""
        self.area1_people = msg.data
    
    def area2_people_callback(self, msg):
        """Callback for Area 2 people count."""
        self.area2_people = msg.data
    
    def area1_time_callback(self, msg):
        """Callback for Area 1 time estimate."""
        self.area1_time = msg.data
    
    def area2_time_callback(self, msg):
        """Callback for Area 2 time estimate."""
        self.area2_time = msg.data
    
    def decision_callback(self, msg):
        """Callback for ethical decision."""
        self.decision = msg.data
        self.get_logger().info(f'Received ethical decision: {self.decision}')
    
    def reasoning_callback(self, msg):
        """Callback for decision reasoning."""
        self.reasoning = msg.data
    
    def robot_state_callback(self, msg):
        """Callback for robot state."""
        self.robot_state = msg.data
    
    def update_plot(self, frame):
        """Update visualization with current data."""
        # Update people count bars
        self.bar_people[0].set_height(self.area1_people)
        self.bar_people[1].set_height(self.area2_people)
        
        # Update time bars
        self.bar_time[0].set_height(self.area1_time)
        self.bar_time[1].set_height(self.area2_time)
        
        # Update decision status
        if self.decision == "area1":
            self.area1_patch.set_alpha(0.8)
            self.area2_patch.set_alpha(0.3)
        elif self.decision == "area2":
            self.area1_patch.set_alpha(0.3)
            self.area2_patch.set_alpha(0.8)
        else:
            self.area1_patch.set_alpha(0.3)
            self.area2_patch.set_alpha(0.3)
        
        # Update reasoning text
        self.reasoning_text.set_text(self.reasoning)
        
        # Update status text
        self.status_text.set_text(f"Robot State: {self.robot_state}")
        
        return self.bar_people + self.bar_time + [self.area1_patch, self.area2_patch, 
                                                 self.reasoning_text, self.status_text]


def main(args=None):
    """Run the decision visualizer."""
    rclpy.init(args=args)
    visualizer = DecisionVisualizer()
    
    # Set up a separate thread for spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,))
    spin_thread.start()
    
    # Show the plot (this blocks until window is closed)
    plt.tight_layout()
    plt.show()
    
    # Shutdown
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()