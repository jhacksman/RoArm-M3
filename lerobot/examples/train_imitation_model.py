#!/usr/bin/env python3
"""
Train Imitation Learning Model Example for RoArm-M3 Pro with LeRobot

This example demonstrates how to train an imitation learning model
using data recorded from the RoArm-M3 Pro.
"""

import os
import json
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

# Configuration
DATASET_PATH = "./datasets/teleoperation/teleop_20250228_120000"  # Replace with your dataset path
MODEL_SAVE_PATH = "./models"
EPOCHS = 100
BATCH_SIZE = 64
LEARNING_RATE = 0.001
SEQUENCE_LENGTH = 10  # Number of timesteps to use as input
PREDICTION_HORIZON = 5  # Number of timesteps to predict

# Create model save directory if it doesn't exist
os.makedirs(MODEL_SAVE_PATH, exist_ok=True)

class JointPositionDataset(Dataset):
    """Dataset for joint position sequences."""
    
    def __init__(self, csv_path, sequence_length=10, prediction_horizon=5):
        """Initialize the dataset.
        
        Args:
            csv_path (str): Path to the CSV file containing joint positions
            sequence_length (int): Number of timesteps to use as input
            prediction_horizon (int): Number of timesteps to predict
        """
        # Load data
        self.data = pd.read_csv(csv_path)
        
        # Extract joint positions (excluding timestamp column)
        self.joint_positions = self.data.iloc[:, 1:].values
        
        # Normalize data
        self.joint_min = np.min(self.joint_positions, axis=0)
        self.joint_max = np.max(self.joint_positions, axis=0)
        self.joint_positions_norm = (self.joint_positions - self.joint_min) / (self.joint_max - self.joint_min + 1e-6)
        
        self.sequence_length = sequence_length
        self.prediction_horizon = prediction_horizon
    
    def __len__(self):
        """Return the number of sequences in the dataset."""
        return max(0, len(self.joint_positions_norm) - self.sequence_length - self.prediction_horizon + 1)
    
    def __getitem__(self, idx):
        """Get a sequence and its target."""
        # Input sequence
        sequence = self.joint_positions_norm[idx:idx+self.sequence_length]
        
        # Target sequence
        target = self.joint_positions_norm[idx+self.sequence_length:idx+self.sequence_length+self.prediction_horizon]
        
        return torch.FloatTensor(sequence), torch.FloatTensor(target)
    
    def get_normalization_params(self):
        """Return the normalization parameters."""
        return self.joint_min, self.joint_max

class ImitationModel(nn.Module):
    """Imitation learning model for predicting joint positions."""
    
    def __init__(self, input_size, hidden_size, output_size, sequence_length, prediction_horizon):
        """Initialize the model.
        
        Args:
            input_size (int): Number of input features (joints)
            hidden_size (int): Size of the hidden layers
            output_size (int): Number of output features (joints)
            sequence_length (int): Number of timesteps in input sequence
            prediction_horizon (int): Number of timesteps to predict
        """
        super(ImitationModel, self).__init__()
        
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.sequence_length = sequence_length
        self.prediction_horizon = prediction_horizon
        
        # LSTM layer
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=2,
            batch_first=True,
            dropout=0.2
        )
        
        # Fully connected layers
        self.fc1 = nn.Linear(hidden_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size * prediction_horizon)
    
    def forward(self, x):
        """Forward pass.
        
        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, sequence_length, input_size)
        
        Returns:
            torch.Tensor: Output tensor of shape (batch_size, prediction_horizon, output_size)
        """
        # LSTM forward pass
        lstm_out, _ = self.lstm(x)
        
        # Use only the last output from the LSTM
        lstm_out = lstm_out[:, -1, :]
        
        # Fully connected layers
        fc1_out = self.relu(self.fc1(lstm_out))
        fc2_out = self.fc2(fc1_out)
        
        # Reshape to (batch_size, prediction_horizon, output_size)
        output = fc2_out.view(-1, self.prediction_horizon, self.output_size)
        
        return output

def train_model(model, train_loader, val_loader, criterion, optimizer, epochs):
    """Train the model.
    
    Args:
        model (nn.Module): The model to train
        train_loader (DataLoader): DataLoader for training data
        val_loader (DataLoader): DataLoader for validation data
        criterion: Loss function
        optimizer: Optimizer
        epochs (int): Number of epochs to train for
    
    Returns:
        dict: Training history
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training on {device}")
    
    model.to(device)
    
    # Training history
    history = {
        'train_loss': [],
        'val_loss': []
    }
    
    # Early stopping parameters
    best_val_loss = float('inf')
    patience = 10
    patience_counter = 0
    
    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        
        for sequences, targets in train_loader:
            sequences, targets = sequences.to(device), targets.to(device)
            
            # Forward pass
            outputs = model(sequences)
            loss = criterion(outputs, targets)
            
            # Backward pass and optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
        
        train_loss /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        
        with torch.no_grad():
            for sequences, targets in val_loader:
                sequences, targets = sequences.to(device), targets.to(device)
                
                # Forward pass
                outputs = model(sequences)
                loss = criterion(outputs, targets)
                
                val_loss += loss.item()
        
        val_loss /= len(val_loader)
        
        # Update history
        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        
        # Print progress
        print(f"Epoch {epoch+1}/{epochs}, Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}")
        
        # Early stopping
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            
            # Save the best model
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'train_loss': train_loss,
                'val_loss': val_loss,
            }, os.path.join(MODEL_SAVE_PATH, 'best_model.pt'))
            
            print(f"Model saved at epoch {epoch+1}")
        else:
            patience_counter += 1
            if patience_counter >= patience:
                print(f"Early stopping at epoch {epoch+1}")
                break
    
    return history

def plot_training_history(history):
    """Plot the training history."""
    plt.figure(figsize=(10, 5))
    plt.plot(history['train_loss'], label='Train Loss')
    plt.plot(history['val_loss'], label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training History')
    plt.legend()
    plt.grid(True)
    
    # Save the plot
    plot_path = os.path.join(MODEL_SAVE_PATH, 'training_history.png')
    plt.savefig(plot_path)
    print(f"Training history plot saved to {plot_path}")
    
    plt.close()

def save_model_metadata(dataset, model):
    """Save metadata about the model and training."""
    metadata = {
        'model_type': 'ImitationModel',
        'input_size': model.input_size,
        'hidden_size': model.hidden_size,
        'output_size': model.output_size,
        'sequence_length': model.sequence_length,
        'prediction_horizon': model.prediction_horizon,
        'joint_min': dataset.joint_min.tolist(),
        'joint_max': dataset.joint_max.tolist(),
        'training_parameters': {
            'epochs': EPOCHS,
            'batch_size': BATCH_SIZE,
            'learning_rate': LEARNING_RATE
        },
        'dataset_path': DATASET_PATH,
        'creation_date': pd.Timestamp.now().isoformat()
    }
    
    metadata_path = os.path.join(MODEL_SAVE_PATH, 'model_metadata.json')
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"Model metadata saved to {metadata_path}")

def main():
    """Main function to run the example."""
    print("RoArm-M3 Pro Imitation Learning Training Example")
    print("----------------------------------------------")
    
    # Check if dataset exists
    csv_path = os.path.join(DATASET_PATH, 'joint_positions.csv')
    if not os.path.exists(csv_path):
        print(f"Dataset not found at {csv_path}")
        print("Please record a dataset first using record_teleoperation.py")
        return
    
    # Load dataset
    print(f"Loading dataset from {csv_path}")
    dataset = JointPositionDataset(
        csv_path=csv_path,
        sequence_length=SEQUENCE_LENGTH,
        prediction_horizon=PREDICTION_HORIZON
    )
    
    # Split dataset into training and validation sets
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)
    
    print(f"Dataset loaded: {len(dataset)} sequences")
    print(f"Training set: {len(train_dataset)} sequences")
    print(f"Validation set: {len(val_dataset)} sequences")
    
    # Create model
    input_size = dataset.joint_positions.shape[1]  # Number of joints
    hidden_size = 128
    output_size = input_size
    
    model = ImitationModel(
        input_size=input_size,
        hidden_size=hidden_size,
        output_size=output_size,
        sequence_length=SEQUENCE_LENGTH,
        prediction_horizon=PREDICTION_HORIZON
    )
    
    # Define loss function and optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    # Train model
    print("Starting training...")
    history = train_model(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        criterion=criterion,
        optimizer=optimizer,
        epochs=EPOCHS
    )
    
    # Plot training history
    plot_training_history(history)
    
    # Save model metadata
    save_model_metadata(dataset, model)
    
    print("\nTraining completed successfully!")
    print(f"Model saved to {os.path.join(MODEL_SAVE_PATH, 'best_model.pt')}")
    print("You can now deploy this model using deploy_imitation_model.py")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
