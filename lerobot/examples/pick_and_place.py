#!/usr/bin/env python3
"""
Pick and Place Example for RoArm-M3 Pro with LeRobot

This example demonstrates how to implement a pick and place task
using the RoArm-M3 Pro robotic arm with LeRobot.
"""

import time
import json
import requests
import numpy as np
import argparse

# Configuration
ARM_IP = "192.168.4.1"  # Default IP when connected to arm's WiFi
HTTP_TIMEOUT = 5  # seconds

# Pick and place positions (in joint space)
HOME_POSITION = [0, 0, 0, 0, 0, 0.5]  # [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
PICK_POSITION = [1.57, 0.7, 0.7, 0, 0, 0.5]  # Position above the object
PICK_DOWN_POSITION = [1.57, 0.9, 0.9, 0, 0, 0.5]  # Position to grasp the object
PLACE_POSITION = [-1.57, 0.7, 0.7, 0, 0, 0.5]  # Position above the place location
PLACE_DOWN_POSITION = [-1.57, 0.9, 0.9, 0, 0, 0.5]  # Position to release the object

def send_command(command):
    """Send a command to the RoArm-M3 Pro via HTTP."""
    url = f"http://{ARM_IP}/js?json={json.dumps(command)}"
    try:
        response = requests.get(url, timeout=HTTP_TIMEOUT)
        return response.text
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

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

def set_gripper(position, speed=50):
    """Set the gripper position.
    
    Args:
        position (float): Gripper position (0.0: closed, 1.0: open)
        speed (int): Movement speed (1-100)
    """
    cmd = {
        "type": "AngleCtrl",
        "id": 6,
        "angle": position,
        "speed": speed
    }
    return send_command(cmd)

def move_to_position(position, speed=30, wait_time=1.0):
    """Move the arm to a specific position in joint space.
    
    Args:
        position (list): Joint angles [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        speed (int): Movement speed (1-100)
        wait_time (float): Time to wait after movement (seconds)
    """
    print(f"Moving to position: {position}")
    for i, angle in enumerate(position[:-1]):  # Exclude gripper
        move_joint(i+1, angle, speed)
    
    # Wait for movement to complete
    time.sleep(wait_time)
    
    # Set gripper position
    set_gripper(position[5], speed)
    time.sleep(wait_time)

def pick_and_place_sequence(cycles=1, speed=30):
    """Execute a pick and place sequence.
    
    Args:
        cycles (int): Number of pick and place cycles to perform
        speed (int): Movement speed (1-100)
    """
    print(f"Starting pick and place sequence ({cycles} cycles)")
    
    for cycle in range(cycles):
        print(f"\nCycle {cycle+1}/{cycles}")
        
        # Move to home position
        print("Moving to home position")
        move_to_position(HOME_POSITION, speed)
        
        # Open gripper
        print("Opening gripper")
        set_gripper(1.0, speed)
        time.sleep(1.0)
        
        # Move to pick position
        print("Moving to pick position")
        move_to_position(PICK_POSITION, speed)
        
        # Move down to grasp object
        print("Moving down to grasp object")
        move_to_position(PICK_DOWN_POSITION, speed)
        
        # Close gripper to grasp object
        print("Closing gripper to grasp object")
        set_gripper(0.2, speed)  # Partially closed to grasp object
        time.sleep(1.0)
        
        # Move up with object
        print("Moving up with object")
        move_to_position(PICK_POSITION, speed)
        
        # Move to place position
        print("Moving to place position")
        move_to_position(PLACE_POSITION, speed)
        
        # Move down to place object
        print("Moving down to place object")
        move_to_position(PLACE_DOWN_POSITION, speed)
        
        # Open gripper to release object
        print("Opening gripper to release object")
        set_gripper(1.0, speed)
        time.sleep(1.0)
        
        # Move up after placing
        print("Moving up after placing")
        move_to_position(PLACE_POSITION, speed)
    
    # Return to home position
    print("\nReturning to home position")
    move_to_position(HOME_POSITION, speed)
    
    print("\nPick and place sequence completed")

def main():
    """Main function to run the example."""
    parser = argparse.ArgumentParser(description='RoArm-M3 Pro Pick and Place Example')
    parser.add_argument('--ip', type=str, default=ARM_IP, help='IP address of the RoArm-M3 Pro')
    parser.add_argument('--cycles', type=int, default=1, help='Number of pick and place cycles')
    parser.add_argument('--speed', type=int, default=30, help='Movement speed (1-100)')
    args = parser.parse_args()
    
    global ARM_IP
    ARM_IP = args.ip
    
    print("RoArm-M3 Pro Pick and Place Example")
    print("-----------------------------------")
    print(f"Arm IP: {ARM_IP}")
    print(f"Cycles: {args.cycles}")
    print(f"Speed: {args.speed}")
    
    # Check connection
    print("\nChecking connection to RoArm-M3 Pro...")
    response = send_command({"type": "GetStatus"})
    if not response:
        print("Failed to connect to RoArm-M3 Pro. Please check your connection.")
        return
    
    print("Connection successful!")
    
    # Execute pick and place sequence
    try:
        pick_and_place_sequence(args.cycles, args.speed)
    except KeyboardInterrupt:
        print("\nSequence interrupted by user")
    except Exception as e:
        print(f"\nError during sequence: {e}")
    finally:
        # Return to a safe position
        print("Returning to home position...")
        move_to_position(HOME_POSITION, 20)
        print("Example completed")

if __name__ == "__main__":
    main()
