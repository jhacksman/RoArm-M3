#!/usr/bin/env python3
"""
Basic HTTP Control Example for RoArm-M3 Pro with LeRobot

This example demonstrates how to control the RoArm-M3 Pro robotic arm
using HTTP communication through the LeRobot framework.
"""

import time
import json
import requests
import numpy as np

# Configuration
ARM_IP = "192.168.4.1"  # Default IP when connected to arm's WiFi
HTTP_TIMEOUT = 5  # seconds

def send_command(command):
    """Send a command to the RoArm-M3 Pro via HTTP."""
    url = f"http://{ARM_IP}/js?json={json.dumps(command)}"
    try:
        response = requests.get(url, timeout=HTTP_TIMEOUT)
        return response.text
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

def get_status():
    """Get the current status of the RoArm-M3 Pro."""
    cmd = {
        "type": "GetStatus"
    }
    return send_command(cmd)

def move_joint(joint_id, angle, speed=50):
    """Move a specific joint to the specified angle.
    
    Args:
        joint_id (int): Joint ID (1-6)
        angle (float): Target angle in radians
        speed (int): Movement speed (1-100)
    """
    cmd = {
        "type": "AngleCtrl",
        "id": joint_id,
        "angle": angle,
        "speed": speed
    }
    return send_command(cmd)

def set_torque(enable=True):
    """Enable or disable torque on all joints."""
    cmd = {
        "type": "TorqueCtrl",
        "enable": enable
    }
    return send_command(cmd)

def home_position(speed=30):
    """Move the arm to the home position."""
    # Home position angles (in radians)
    home_angles = [0, 0, 0, 0, 0, 0.5]  # Last value is gripper (0-1)
    
    for i, angle in enumerate(home_angles):
        move_joint(i+1, angle, speed)
        time.sleep(0.5)  # Wait for movement to complete
    
    print("Arm moved to home position")

def wave_demo():
    """Demonstrate a simple waving motion."""
    print("Starting wave demo...")
    
    # Move to starting position
    move_joint(1, 0, 50)      # Base
    move_joint(2, 0.5, 50)    # Shoulder
    move_joint(3, 0.5, 50)    # Elbow
    move_joint(4, 0, 50)      # Wrist pitch
    move_joint(5, 0, 50)      # Wrist roll
    time.sleep(2)
    
    # Perform waving motion
    for _ in range(3):
        move_joint(5, 0.7, 70)  # Wave right
        time.sleep(0.5)
        move_joint(5, -0.7, 70) # Wave left
        time.sleep(0.5)
    
    # Return to neutral position
    move_joint(5, 0, 50)
    time.sleep(1)
    
    print("Wave demo completed")

def gripper_test():
    """Test the gripper by opening and closing it."""
    print("Testing gripper...")
    
    # Open gripper
    move_joint(6, 1.0, 50)
    print("Gripper opened")
    time.sleep(1)
    
    # Close gripper
    move_joint(6, 0.0, 50)
    print("Gripper closed")
    time.sleep(1)
    
    # Half open (neutral)
    move_joint(6, 0.5, 50)
    print("Gripper at neutral position")
    
    print("Gripper test completed")

def joint_sweep_test():
    """Test each joint by sweeping through its range of motion."""
    print("Starting joint sweep test...")
    
    # Joint limits (approximate, in radians)
    joint_limits = [
        [-3.14, 3.14],  # Base
        [-1.57, 1.57],  # Shoulder
        [-1.57, 1.57],  # Elbow
        [-1.57, 1.57],  # Wrist pitch
        [-3.14, 3.14],  # Wrist roll
        [0.0, 1.0]      # Gripper
    ]
    
    # Move to home position first
    home_position()
    time.sleep(1)
    
    # Test each joint
    for joint_id in range(1, 7):
        print(f"Testing Joint {joint_id}...")
        
        # Get limits for this joint
        min_angle, max_angle = joint_limits[joint_id-1]
        
        # Move to minimum position
        move_joint(joint_id, min_angle, 30)
        time.sleep(1.5)
        
        # Move to maximum position
        move_joint(joint_id, max_angle, 30)
        time.sleep(1.5)
        
        # Return to middle position
        middle_angle = (min_angle + max_angle) / 2
        move_joint(joint_id, middle_angle, 30)
        time.sleep(1.5)
    
    print("Joint sweep test completed")

def main():
    """Main function to run the example."""
    print("RoArm-M3 Pro Basic HTTP Control Example")
    print("---------------------------------------")
    
    # Check connection
    print("Checking connection to RoArm-M3 Pro...")
    status = get_status()
    if status:
        print(f"Connected successfully. Status: {status}")
    else:
        print("Failed to connect to RoArm-M3 Pro. Please check your connection.")
        return
    
    # Enable torque
    print("Enabling torque...")
    set_torque(True)
    time.sleep(0.5)
    
    # Move to home position
    print("Moving to home position...")
    home_position()
    time.sleep(1)
    
    # Run demos
    gripper_test()
    time.sleep(1)
    
    wave_demo()
    time.sleep(1)
    
    joint_sweep_test()
    time.sleep(1)
    
    # Return to home position
    print("Returning to home position...")
    home_position()
    
    print("Example completed successfully!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExample interrupted by user")
        # Try to stop the arm safely
        set_torque(False)
        print("Torque disabled for safety")
    except Exception as e:
        print(f"Error: {e}")
        # Try to stop the arm safely
        set_torque(False)
        print("Torque disabled for safety")
