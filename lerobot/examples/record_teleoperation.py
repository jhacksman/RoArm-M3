#!/usr/bin/env python3
"""
Record Teleoperation Example for RoArm-M3 Pro with LeRobot

This example demonstrates how to record arm movements during teleoperation
for later use in training LeRobot models.
"""

import os
import time
import json
import requests
import numpy as np
import datetime
import threading
import csv

# Configuration
ARM_IP = "192.168.4.1"  # Default IP when connected to arm's WiFi
HTTP_TIMEOUT = 5  # seconds
RECORDING_FREQUENCY = 10  # Hz
RECORDING_DURATION = 60  # seconds (set to None for indefinite recording)
SAVE_DIRECTORY = "./datasets/teleoperation"

def send_command(command):
    """Send a command to the RoArm-M3 Pro via HTTP."""
    url = f"http://{ARM_IP}/js?json={json.dumps(command)}"
    try:
        response = requests.get(url, timeout=HTTP_TIMEOUT)
        return response.text
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

def get_joint_positions():
    """Get the current joint positions of the RoArm-M3 Pro."""
    cmd = {
        "type": "GetJointPositions"
    }
    response = send_command(cmd)
    if response:
        try:
            data = json.loads(response)
            return data.get("positions", [])
        except json.JSONDecodeError:
            print("Error decoding joint positions")
    return None

def set_torque(enable=True):
    """Enable or disable torque on all joints."""
    cmd = {
        "type": "TorqueCtrl",
        "enable": enable
    }
    return send_command(cmd)

def create_dataset_directory():
    """Create a directory for saving the dataset."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_dir = os.path.join(SAVE_DIRECTORY, f"teleop_{timestamp}")
    os.makedirs(dataset_dir, exist_ok=True)
    return dataset_dir

def record_data(dataset_dir, stop_event):
    """Record joint positions at regular intervals."""
    print(f"Recording data to {dataset_dir}")
    
    # Create CSV file for joint positions
    csv_path = os.path.join(dataset_dir, "joint_positions.csv")
    with open(csv_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Write header
        header = ["timestamp", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        csv_writer.writerow(header)
        
        # Record data
        start_time = time.time()
        sample_count = 0
        
        while not stop_event.is_set():
            # Get current time
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if recording duration has been reached
            if RECORDING_DURATION is not None and elapsed_time >= RECORDING_DURATION:
                print(f"Recording duration ({RECORDING_DURATION}s) reached")
                break
            
            # Get joint positions
            positions = get_joint_positions()
            if positions:
                # Write to CSV
                row = [elapsed_time] + positions
                csv_writer.writerow(row)
                
                # Update progress
                sample_count += 1
                if sample_count % 10 == 0:
                    print(f"Recorded {sample_count} samples ({elapsed_time:.1f}s elapsed)")
            else:
                print("Failed to get joint positions")
            
            # Sleep to maintain recording frequency
            time_to_sleep = (1.0 / RECORDING_FREQUENCY) - (time.time() - current_time)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
    
    print(f"Recording completed. Saved {sample_count} samples to {csv_path}")
    return sample_count

def create_metadata_file(dataset_dir, sample_count):
    """Create a metadata file with information about the dataset."""
    metadata_path = os.path.join(dataset_dir, "metadata.json")
    metadata = {
        "dataset_name": os.path.basename(dataset_dir),
        "creation_date": datetime.datetime.now().isoformat(),
        "arm_type": "RoArm-M3 Pro",
        "recording_frequency": RECORDING_FREQUENCY,
        "recording_duration": RECORDING_DURATION,
        "sample_count": sample_count,
        "joint_count": 6,
        "joint_names": ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
    }
    
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"Metadata saved to {metadata_path}")

def main():
    """Main function to run the example."""
    print("RoArm-M3 Pro Teleoperation Recording Example")
    print("-------------------------------------------")
    
    # Check connection
    print("Checking connection to RoArm-M3 Pro...")
    response = send_command({"type": "GetStatus"})
    if not response:
        print("Failed to connect to RoArm-M3 Pro. Please check your connection.")
        return
    
    # Create dataset directory
    dataset_dir = create_dataset_directory()
    
    # Disable torque for manual positioning
    print("Disabling torque for teleoperation...")
    set_torque(False)
    
    # Create stop event for threading
    stop_event = threading.Event()
    
    # Start recording in a separate thread
    print("Starting recording...")
    print("Move the arm manually to demonstrate the desired movements.")
    print(f"Recording will run for {RECORDING_DURATION} seconds.")
    print("Press Ctrl+C to stop recording early.")
    
    try:
        sample_count = record_data(dataset_dir, stop_event)
        
        # Create metadata file
        create_metadata_file(dataset_dir, sample_count)
        
        print("\nRecording completed successfully!")
        print(f"Dataset saved to {dataset_dir}")
        print("You can now use this dataset to train a LeRobot model.")
        
    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
        stop_event.set()
    finally:
        # Re-enable torque
        print("Re-enabling torque...")
        set_torque(True)

if __name__ == "__main__":
    main()
