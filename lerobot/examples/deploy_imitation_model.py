#!/usr/bin/env python3
"""
Deploy Imitation Learning Model Example for RoArm-M3 Pro with LeRobot

This example demonstrates how to deploy a trained imitation learning model
to control the RoArm-M3 Pro robotic arm.
"""

import os
import json
import time
import numpy as np
import torch
import requests
from collections import deque

# Configuration
MODEL_PATH = "./models/best_model.pt"
METADATA_PATH = "./models/model_metadata.json"
ARM_IP = "192.168.4.1"  # Default IP when connected to arm's WiFi
HTTP_TIMEOUT = 5  # seconds
CONTROL_FREQUENCY = 10  # Hz
SEQUENCE_LENGTH = 10  # Must match the model's sequence length
SAFETY_LIMITS = True  # Enable safety limits on joint movements

def load_model_and_metadata():
    """Load the trained model and its metadata."""
    print(f"Loading model from {MODEL_PATH}")
    
    # Check if model exists
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found at {MODEL_PATH}")
    
    # Check if metadata exists
    if not os.path.exists(METADATA_PATH):
        raise FileNotFoundError(f"Metadata not found at {METADATA_PATH}")
    
    # Load metadata
    with open(METADATA_PATH, 'r') as f:
        metadata = json.load(f)
    
    # Load model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(MODEL_PATH, map_location=device)
    
    # Recreate model architecture
    from train_imitation_model import ImitationModel
    
    model = ImitationModel(
        input_size=metadata['input_size'],
        hidden_size=metadata['hidden_size'],
        output_size=metadata['output_size'],
        sequence_length=metadata['sequence_length'],
        prediction_horizon=metadata['prediction_horizon']
    )
    
    # Load model weights
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()
    
    print(f"Model loaded successfully (trained for {checkpoint['epoch']+1} epochs)")
    print(f"Validation loss: {checkpoint['val_loss']:.4f}")
    
    return model, metadata, device

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

def normalize_joint_positions(positions, joint_min, joint_max):
    """Normalize joint positions using the model's normalization parameters."""
    return (positions - joint_min) / (joint_max - joint_min + 1e-6)

def denormalize_joint_positions(normalized_positions, joint_min, joint_max):
    """Denormalize joint positions using the model's normalization parameters."""
    return normalized_positions * (joint_max - joint_min + 1e-6) + joint_min

def apply_safety_limits(positions, prev_positions, max_delta=0.1):
    """Apply safety limits to joint positions to prevent sudden movements.
    
    Args:
        positions (list): Target joint positions
        prev_positions (list): Previous joint positions
        max_delta (float): Maximum allowed change in joint position (radians)
    
    Returns:
        list: Safe joint positions
    """
    if prev_positions is None:
        return positions
    
    safe_positions = []
    for i, (pos, prev_pos) in enumerate(zip(positions, prev_positions)):
        delta = pos - prev_pos
        if abs(delta) > max_delta:
            # Limit the change to max_delta
            safe_pos = prev_pos + np.sign(delta) * max_delta
            print(f"Safety limit applied to joint {i+1}: {pos:.2f} -> {safe_pos:.2f}")
        else:
            safe_pos = pos
        safe_positions.append(safe_pos)
    
    return safe_positions

def main():
    """Main function to run the example."""
    print("RoArm-M3 Pro Imitation Learning Deployment Example")
    print("------------------------------------------------")
    
    # Load model and metadata
    try:
        model, metadata, device = load_model_and_metadata()
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    # Extract normalization parameters
    joint_min = np.array(metadata['joint_min'])
    joint_max = np.array(metadata['joint_max'])
    
    # Check connection to arm
    print("Checking connection to RoArm-M3 Pro...")
    response = send_command({"type": "GetStatus"})
    if not response:
        print("Failed to connect to RoArm-M3 Pro. Please check your connection.")
        return
    
    # Enable torque
    print("Enabling torque...")
    set_torque(True)
    time.sleep(0.5)
    
    # Initialize sequence buffer
    sequence_buffer = deque(maxlen=SEQUENCE_LENGTH)
    
    # Initialize with current joint positions
    print("Initializing sequence buffer with current joint positions...")
    current_positions = get_joint_positions()
    if not current_positions:
        print("Failed to get current joint positions")
        return
    
    # Normalize current positions
    normalized_positions = normalize_joint_positions(
        np.array(current_positions),
        joint_min,
        joint_max
    )
    
    # Fill buffer with current positions
    for _ in range(SEQUENCE_LENGTH):
        sequence_buffer.append(normalized_positions)
    
    # Previous positions for safety checks
    prev_positions = current_positions
    
    print("\nStarting model-based control...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            start_time = time.time()
            
            # Convert sequence buffer to tensor
            sequence = np.array(sequence_buffer)
            sequence_tensor = torch.FloatTensor(sequence).unsqueeze(0).to(device)
            
            # Get model prediction
            with torch.no_grad():
                prediction = model(sequence_tensor)
            
            # Extract next position (first step of prediction)
            next_normalized_position = prediction[0, 0].cpu().numpy()
            
            # Denormalize to get actual joint positions
            next_position = denormalize_joint_positions(
                next_normalized_position,
                joint_min,
                joint_max
            )
            
            # Apply safety limits if enabled
            if SAFETY_LIMITS:
                next_position = apply_safety_limits(next_position, prev_positions)
            
            # Send commands to move joints
            for i, angle in enumerate(next_position):
                move_joint(i+1, float(angle), speed=30)
            
            # Update previous positions
            prev_positions = next_position
            
            # Get new joint positions
            time.sleep(0.1)  # Give time for movement
            current_positions = get_joint_positions()
            if current_positions:
                # Normalize current positions
                normalized_positions = normalize_joint_positions(
                    np.array(current_positions),
                    joint_min,
                    joint_max
                )
                
                # Update sequence buffer
                sequence_buffer.append(normalized_positions)
            
            # Maintain control frequency
            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1.0 / CONTROL_FREQUENCY) - elapsed_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Print actual control frequency
            actual_frequency = 1.0 / (time.time() - start_time)
            print(f"Control frequency: {actual_frequency:.1f} Hz", end="\r")
    
    except KeyboardInterrupt:
        print("\n\nControl interrupted by user")
    except Exception as e:
        print(f"\n\nError during control: {e}")
    finally:
        # Return to a safe position
        print("Returning to home position...")
        home_positions = [0, 0, 0, 0, 0, 0.5]  # Home position
        for i, angle in enumerate(home_positions):
            move_joint(i+1, angle, speed=20)
        
        print("Deployment completed")

if __name__ == "__main__":
    main()
