#!/usr/bin/env python3
"""
SmartSort Warehouse Robot - Main State Machine
ROS2 Jazzy Implementation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import time

class RobotState(Enum):
    """Robot operational states"""
    SEARCHING = 1      # Looking for packages
    APPROACHING = 2    # Moving toward detected package
    PICKING = 3        # Grabbing the package
    NAVIGATING = 4     # Going to drop zone
    DROPPING = 5       # Releasing package
    RETURNING = 6      # Going back to search area

class ColorZone:
    """Package color to zone mapping"""
    RED = "Zone_A"
    BLUE = "Zone_B"
    GREEN = "Zone_C"
    
    # Zone coordinates (x, y, z)
    ZONES = {
        "Zone_A": (5.0, 2.0, 0.0),   # Red packages
        "Zone_B": (5.0, 0.0, 0.0),   # Blue packages
        "Zone_C": (5.0, -2.0, 0.0),  # Green packages
    }
    
    SEARCH_AREA = (0.0, 0.0, 0.0)  # Starting/search position

class SmartSortRobot(Node):
    """Main robot controller with state machine"""
    
    def __init__(self):
        super().__init__('smartsort_robot')
        
        # State initialization
        self.state = RobotState.SEARCHING
        self.detected_color = None
        self.target_zone = None
        self.package_detected = False
        self.package_in_gripper = False
        
        # ROS2 Communication
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        
        # State machine timer
        self.state_timer = self.create_timer(0.1, self.state_machine_loop)
        
        # Logging
        self.get_logger().info('SmartSort Robot Initialized')
        self.get_logger().info(f'Current State: {self.state.name}')
        
    def camera_callback(self, msg):
        """Process camera images for package detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect colored packages
            color, centroid = self.detect_package_color(cv_image)
            
            if color:
                self.detected_color = color
                self.package_detected = True
                self.get_logger().info(f'Package detected: {color}')
            else:
                self.package_detected = False
                
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {str(e)}')
    
    def detect_package_color(self, image):
        """
        Color detection using HSV color space
        Returns: (color_name, centroid) or (None, None)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Color ranges in HSV
        color_ranges = {
            'RED': ([0, 120, 70], [10, 255, 255]),      # Red hue (lower)
            'RED2': ([170, 120, 70], [180, 255, 255]),  # Red hue (upper)
            'BLUE': ([100, 120, 70], [130, 255, 255]),  # Blue
            'GREEN': ([40, 120, 70], [80, 255, 255]),   # Green
        }
        
        detected_color = None
        largest_area = 500  # Minimum area threshold
        centroid = None
        
        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for color
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_area:
                    largest_area = area
                    detected_color = color_name.replace('2', '')  # Handle RED2
                    
                    # Calculate centroid
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        centroid = (cx, cy)
        
        return detected_color, centroid
    
    def state_machine_loop(self):
        """Main state machine logic"""
        
        if self.state == RobotState.SEARCHING:
            self.searching_behavior()
            
        elif self.state == RobotState.APPROACHING:
            self.approaching_behavior()
            
        elif self.state == RobotState.PICKING:
            self.picking_behavior()
            
        elif self.state == RobotState.NAVIGATING:
            self.navigating_behavior()
            
        elif self.state == RobotState.DROPPING:
            self.dropping_behavior()
            
        elif self.state == RobotState.RETURNING:
            self.returning_behavior()
    
    def searching_behavior(self):
        """Search pattern: rotate slowly to scan for packages"""
        if self.package_detected:
            # Package found, switch to approaching
            self.stop_robot()
            self.transition_state(RobotState.APPROACHING)
        else:
            # Continue rotating to search
            twist = Twist()
            twist.angular.z = 0.3  # Rotate slowly
            self.cmd_vel_pub.publish(twist)
    
    def approaching_behavior(self):
        """Move toward detected package"""
        if not self.package_detected:
            # Lost sight of package, go back to searching
            self.transition_state(RobotState.SEARCHING)
            return
        
        # Move forward slowly
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)
        
        # Simulate distance check (in real system, use depth sensor)
        time.sleep(2.0)  # Simple simulation
        
        # Close enough, start picking
        self.stop_robot()
        self.transition_state(RobotState.PICKING)
    
    def picking_behavior(self):
        """Activate gripper to grab package"""
        self.get_logger().info('Closing gripper...')
        
        # Send gripper close command
        gripper_msg = String()
        gripper_msg.data = 'close'
        self.gripper_pub.publish(gripper_msg)
        
        time.sleep(2.0)  # Wait for gripper
        self.package_in_gripper = True
        
        # Determine target zone based on color
        if self.detected_color == 'RED':
            self.target_zone = ColorZone.ZONES['Zone_A']
        elif self.detected_color == 'BLUE':
            self.target_zone = ColorZone.ZONES['Zone_B']
        elif self.detected_color == 'GREEN':
            self.target_zone = ColorZone.ZONES['Zone_C']
        
        self.get_logger().info(f'Package secured. Target: {self.target_zone}')
        self.transition_state(RobotState.NAVIGATING)
    
    def navigating_behavior(self):
        """Navigate to target drop zone"""
        if not self.package_in_gripper:
            self.get_logger().warn('Lost package during navigation!')
            self.transition_state(RobotState.SEARCHING)
            return
        
        # Simple navigation (in real system, use Nav2)
        self.get_logger().info(f'Navigating to {self.target_zone}...')
        
        # Simulate navigation
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(5.0)  # Simulate travel time
        
        self.stop_robot()
        self.transition_state(RobotState.DROPPING)
    
    def dropping_behavior(self):
        """Release package at zone"""
        self.get_logger().info('Opening gripper to drop package...')
        
        # Send gripper open command
        gripper_msg = String()
        gripper_msg.data = 'open'
        self.gripper_pub.publish(gripper_msg)
        
        time.sleep(2.0)
        self.package_in_gripper = False
        self.detected_color = None
        
        self.get_logger().info('Package delivered successfully!')
        self.transition_state(RobotState.RETURNING)
    
    def returning_behavior(self):
        """Return to search area"""
        self.get_logger().info('Returning to search area...')
        
        # Navigate back to start
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(5.0)  # Simulate return
        
        self.stop_robot()
        self.transition_state(RobotState.SEARCHING)
        self.get_logger().info('Ready for next package!')
    
    def transition_state(self, new_state):
        """Change robot state with logging"""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')
    
    def stop_robot(self):
        """Stop all motion"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    robot = SmartSortRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
