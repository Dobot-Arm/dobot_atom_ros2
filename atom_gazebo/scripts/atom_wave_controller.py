#!/usr/bin/env python3
"""
Atom Robot Wave Controller
This script controls the Atom robot to perform a waving motion using the right arm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np

class AtomWaveController(Node):
    def __init__(self):
        super().__init__('atom_wave_controller')
        
        # Publishers for joint commands
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_position_controller/commands', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Arm joint names (in order for controller)
        self.arm_joints = [
            # Left arm
            'uidx_l_arm_joint1',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint', 
            'uidx_l_arm_joint2',
            'left_elbow_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            # Right arm
            'uidx_r_arm_joint1',
            'right_shoulder_roll_joint', 
            'right_shoulder_yaw_joint',
            'uidx_r_arm_joint2',
            'right_elbow_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint'
        ]
        
        # Right arm joint indices in the combined arm controller
        self.right_arm_indices = [7, 8, 9, 10, 11, 12, 13]
        
        # Current joint positions
        self.current_positions = {}
        
        # Wave motion parameters
        self.wave_amplitude = 0.8  # Amplitude of waving motion (radians)
        self.wave_frequency = 1.0  # Frequency of waving (Hz)
        self.wave_duration = 10.0  # Total duration of waving (seconds)
        
        # Initial positions for all arm joints (left arm at rest, right arm waving)
        self.initial_wave_positions = [
            # Left arm (rest position)
            0.0,  # uidx_l_arm_joint1
            0.0,  # left_shoulder_roll_joint
            0.0,  # left_shoulder_yaw_joint
            0.0,  # uidx_l_arm_joint2
            0.0,  # left_elbow_roll_joint
            0.0,  # left_wrist_pitch_joint
            0.0,  # left_wrist_yaw_joint
            # Right arm (wave position)
            -0.5, # uidx_r_arm_joint1 (shoulder pitch - raise arm)
            -1.2, # right_shoulder_roll_joint (shoulder roll - move arm out)
            0.0,  # right_shoulder_yaw_joint (shoulder yaw - neutral)
            -0.3, # uidx_r_arm_joint2 (elbow pitch - slight bend)
            0.0,  # right_elbow_roll_joint (elbow roll - neutral)
            0.0,  # right_wrist_pitch_joint (wrist pitch - neutral)
            0.0   # right_wrist_yaw_joint (wrist yaw - neutral)
        ]
        
        # Rest positions (all joints to 0)
        self.rest_positions = [0.0] * len(self.arm_joints)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        # State machine
        self.state = 'idle'  # 'idle', 'moving_to_wave', 'waving', 'returning_to_rest'
        self.start_time = None
        self.motion_start_time = None
        
        self.get_logger().info('Atom Wave Controller initialized')
        self.get_logger().info('Commands:')
        self.get_logger().info('  - Call start_wave() to begin waving')
        self.get_logger().info('  - Call stop_wave() to return to rest position')
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if name in self.arm_joints:
                self.current_positions[name] = msg.position[i]
    
    def publish_joint_commands(self, target_positions):
        """Publish joint position commands"""
        cmd_msg = Float64MultiArray()
        if isinstance(target_positions, list):
            cmd_msg.data = target_positions
        else:
            # Convert dict to list in correct order
            cmd_msg.data = [target_positions.get(joint, 0.0) for joint in self.arm_joints]
        self.arm_cmd_pub.publish(cmd_msg)
        
    def interpolate_positions(self, start_pos, end_pos, t):
        """Interpolate between start and end positions"""
        t = max(0.0, min(1.0, t))  # Clamp t to [0, 1]
        if isinstance(start_pos, list) and isinstance(end_pos, list):
            result = []
            for i in range(len(start_pos)):
                result.append(start_pos[i] + t * (end_pos[i] - start_pos[i]))
            return result
        else:
            result = {}
            for joint in start_pos:
                if joint in end_pos:
                    result[joint] = start_pos[joint] + t * (end_pos[joint] - start_pos[joint])
                else:
                    result[joint] = start_pos[joint]
            return result
    
    def generate_wave_motion(self, t):
        """Generate waving motion based on time"""
        # Base position is the initial wave position
        wave_positions = self.initial_wave_positions.copy()
        
        # Add sinusoidal motion to shoulder yaw for waving (index 9 for right_shoulder_yaw_joint)
        wave_offset = self.wave_amplitude * math.sin(2 * math.pi * self.wave_frequency * t)
        wave_positions[9] += wave_offset
        
        # Add slight elbow motion for more natural waving (index 10 for uidx_r_arm_joint2)
        elbow_offset = 0.2 * math.sin(2 * math.pi * self.wave_frequency * t + math.pi/4)
        wave_positions[10] += elbow_offset
        
        return wave_positions
    
    def control_callback(self):
        """Main control loop"""
        current_time = time.time()
        
        if self.state == 'idle':
            # Do nothing, wait for command
            pass
            
        elif self.state == 'moving_to_wave':
            if self.motion_start_time is None:
                self.motion_start_time = current_time
                self.get_logger().info('Moving to wave position...')
            
            # Move to initial wave position over 2 seconds
            elapsed = current_time - self.motion_start_time
            move_duration = 2.0
            
            if elapsed < move_duration:
                t = elapsed / move_duration
                target_positions = self.interpolate_positions(
                    self.rest_positions, 
                    self.initial_wave_positions, 
                    t
                )
                self.publish_joint_commands(target_positions)
            else:
                # Transition to waving
                self.state = 'waving'
                self.motion_start_time = current_time
                self.get_logger().info('Starting wave motion!')
                
        elif self.state == 'waving':
            if self.motion_start_time is None:
                self.motion_start_time = current_time
            
            # Perform waving motion
            elapsed = current_time - self.motion_start_time
            
            if elapsed < self.wave_duration:
                wave_positions = self.generate_wave_motion(elapsed)
                self.publish_joint_commands(wave_positions)
            else:
                # Transition to returning to rest
                self.state = 'returning_to_rest'
                self.motion_start_time = current_time
                self.get_logger().info('Returning to rest position...')
                
        elif self.state == 'returning_to_rest':
            if self.motion_start_time is None:
                self.motion_start_time = current_time
            
            # Return to rest position over 2 seconds
            elapsed = current_time - self.motion_start_time
            move_duration = 2.0
            
            if elapsed < move_duration:
                t = elapsed / move_duration
                target_positions = self.interpolate_positions(
                    self.initial_wave_positions, 
                    self.rest_positions, 
                    t
                )
                self.publish_joint_commands(target_positions)
            else:
                # Return to idle
                self.state = 'idle'
                self.motion_start_time = None
                self.get_logger().info('Wave motion completed!')
    
    def start_wave(self):
        """Start the waving motion"""
        if self.state == 'idle':
            self.state = 'moving_to_wave'
            self.motion_start_time = None
            self.get_logger().info('Starting wave sequence...')
        else:
            self.get_logger().warn('Already in motion, please wait...')
    
    def stop_wave(self):
        """Stop waving and return to rest"""
        if self.state in ['moving_to_wave', 'waving']:
            self.state = 'returning_to_rest'
            self.motion_start_time = None
            self.get_logger().info('Stopping wave motion...')
        else:
            self.get_logger().info('Not currently waving')
    
    def go_to_rest(self):
        """Move to rest position"""
        self.publish_joint_commands(self.rest_positions)
        self.get_logger().info('Moving to rest position')

def main(args=None):
    rclpy.init(args=args)
    
    controller = AtomWaveController()
    
    # Start waving automatically after 3 seconds
    def delayed_start():
        time.sleep(3.0)
        controller.start_wave()
    
    import threading
    start_thread = threading.Thread(target=delayed_start)
    start_thread.daemon = True
    start_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down wave controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()