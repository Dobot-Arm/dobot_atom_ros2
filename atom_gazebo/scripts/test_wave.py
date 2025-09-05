#!/usr/bin/env python3
"""
Test script for Atom robot waving motion
This script demonstrates how to use the AtomWaveController
"""

import rclpy
from atom_wave_controller import AtomWaveController
import time

def main():
    """Main function to test the waving motion"""
    rclpy.init()
    
    # Create the wave controller node
    wave_controller = AtomWaveController()
    
    try:
        print("Starting Atom wave controller test...")
        print("The robot will wave for 10 seconds, then rest for 5 seconds.")
        print("Press Ctrl+C to stop.")
        
        # Start waving
        wave_controller.start_wave()
        print("Robot is now waving!")
        
        # Let it wave for 10 seconds
        time.sleep(10)
        
        # Stop waving
        wave_controller.stop_wave()
        print("Robot stopped waving, returning to rest position.")
        
        # Keep the node alive for a bit to see the rest position
        time.sleep(5)
        
        print("Test completed successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        wave_controller.stop_wave()
    
    finally:
        # Cleanup
        wave_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()