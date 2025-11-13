#!/usr/bin/env python3
"""
SmartSort Gripper Controller
Controls the robot gripper for pick and place operations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import time

class GripperController(Node):
    """Controls gripper opening and closing"""
    
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Gripper state
        self.gripper_state = 'open'  # 'open' or 'closed'
        self.gripper_position = 0.04  # Starting position (open)
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_commands', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/gripper_status',
            10
        )
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            String,
            '/gripper_command',
            self.command_callback,
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info('Gripper Controller initialized')
    
    def command_callback(self, msg):
        """Handle gripper commands"""
        command = msg.data.lower()
        
        if command == 'close':
            self.close_gripper()
        elif command == 'open':
            self.open_gripper()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def close_gripper(self):
        """Close gripper to grab object"""
        if self.gripper_state == 'closed':
            self.get_logger().info('Gripper already closed')
            return
        
        self.get_logger().info('Closing gripper...')
        
        # Target positions for gripper fingers
        # Left finger moves to 0 (toward center)
        # Right finger moves to 0 (toward center)
        target_positions = [0.0, 0.0]
        
        # Gradually move to target
        steps = 20
        for i in range(steps):
            progress = (i + 1) / steps
            current_left = 0.04 * (1 - progress)
            current_right = -0.04 * (1 - progress)
            
            joint_cmd = Float64MultiArray()
            joint_cmd.data = [current_left, current_right]
            self.joint_cmd_pub.publish(joint_cmd)
            
            time.sleep(0.05)
        
        self.gripper_state = 'closed'
        self.gripper_position = 0.0
        self.get_logger().info('Gripper closed')
    
    def open_gripper(self):
        """Open gripper to release object"""
        if self.gripper_state == 'open':
            self.get_logger().info('Gripper already open')
            return
        
        self.get_logger().info('Opening gripper...')
        
        # Target positions for open state
        target_positions = [0.04, -0.04]
        
        # Gradually move to target
        steps = 20
        for i in range(steps):
            progress = (i + 1) / steps
            current_left = 0.04 * progress
            current_right = -0.04 * progress
            
            joint_cmd = Float64MultiArray()
            joint_cmd.data = [current_left, current_right]
            self.joint_cmd_pub.publish(joint_cmd)
            
            time.sleep(0.05)
        
        self.gripper_state = 'open'
        self.gripper_position = 0.04
        self.get_logger().info('Gripper opened')
    
    def publish_status(self):
        """Publish current gripper status"""
        status_msg = String()
        status_msg.data = f'state:{self.gripper_state},position:{self.gripper_position:.3f}'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GripperController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
