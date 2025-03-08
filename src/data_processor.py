"""
Data Processing Module for RoArm-M3 Isaac Sim Bridge

This module handles the processing of telemetry data from RoArm-M3 devices.
It provides data buffering, smoothing, and history tracking capabilities.
"""

import json
import time
import logging
import numpy as np
from typing import Dict, List, Optional, Tuple
from collections import deque

class TelemetryProcessor:
    """
    Processes telemetry data from RoArm-M3 devices
    """
    def __init__(self, config):
        """
        Initialize the TelemetryProcessor
        
        Args:
            config: Configuration object with settings
        """
        self.config = config
        self.logger = logging.getLogger("TelemetryProcessor")
        self.data_buffers = {}  # Dictionary to store data buffers for each device
    
    def process_telemetry(self, device_id: str, raw_data: str) -> Optional[Dict]:
        """
        Process raw telemetry data from a device
        
        Args:
            device_id: The ID of the device that sent the data
            raw_data: The raw JSON data string from the device
            
        Returns:
            Processed data dictionary or None if data is invalid
        """
        try:
            # Parse JSON data
            data = json.loads(raw_data)
            
            # Initialize buffer for this device if it doesn't exist
            if device_id not in self.data_buffers:
                self.data_buffers[device_id] = {
                    field: deque(maxlen=self.config.BUFFER_SIZE) 
                    for field in self.config.TELEMETRY_FIELDS
                }
                # Add timestamp buffer
                self.data_buffers[device_id]["timestamp"] = deque(maxlen=self.config.BUFFER_SIZE)
            
            # Add current data to buffers
            for key in data:
                if key in self.data_buffers[device_id]:
                    self.data_buffers[device_id][key].append(data[key])
            
            # Add timestamp
            current_time = time.time()
            self.data_buffers[device_id]["timestamp"].append(current_time)
            
            # Apply smoothing if needed
            if self.config.SMOOTHING_FACTOR > 0 and len(self.data_buffers[device_id]["timestamp"]) > 1:
                for key in data:
                    if key in self.data_buffers[device_id] and key != "timestamp" and key != "T":
                        # Only apply smoothing to numeric fields
                        if isinstance(data[key], (int, float)):
                            previous_value = list(self.data_buffers[device_id][key])[-2]
                            data[key] = self._apply_smoothing(
                                data[key], 
                                previous_value, 
                                self.config.SMOOTHING_FACTOR
                            )
            
            # Add metadata
            data["device_id"] = device_id
            data["timestamp"] = current_time
            
            return data
            
        except json.JSONDecodeError:
            self.logger.warning(f"Invalid JSON data from device {device_id}: {raw_data}")
            return None
        except Exception as e:
            self.logger.error(f"Error processing data from device {device_id}: {str(e)}")
            return None
    
    def _apply_smoothing(self, current_value, previous_value, alpha):
        """
        Apply exponential smoothing to a value
        
        Args:
            current_value: The current value
            previous_value: The previous value
            alpha: The smoothing factor (0-1)
            
        Returns:
            The smoothed value
        """
        return alpha * current_value + (1 - alpha) * previous_value
    
    def get_device_data_history(self, device_id: str) -> Optional[Dict]:
        """
        Get the data history for a device
        
        Args:
            device_id: The ID of the device to get history for
            
        Returns:
            The data history dictionary or None if not found
        """
        return self.data_buffers.get(device_id)
    
    def get_latest_data(self, device_id: str) -> Optional[Dict]:
        """
        Get the latest data for a device
        
        Args:
            device_id: The ID of the device to get data for
            
        Returns:
            The latest data dictionary or None if not found
        """
        if device_id not in self.data_buffers:
            return None
        
        # Create a dictionary with the latest values
        latest_data = {}
        for key, buffer in self.data_buffers[device_id].items():
            if buffer:  # Check if buffer is not empty
                latest_data[key] = buffer[-1]
        
        return latest_data
    
    def calculate_statistics(self, device_id: str) -> Optional[Dict]:
        """
        Calculate statistics for a device's data
        
        Args:
            device_id: The ID of the device to calculate statistics for
            
        Returns:
            Dictionary of statistics or None if not found
        """
        if device_id not in self.data_buffers:
            return None
        
        stats = {}
        for key, buffer in self.data_buffers[device_id].items():
            if buffer and key != "timestamp" and key != "T":
                # Only calculate stats for numeric fields
                try:
                    values = np.array(list(buffer))
                    stats[key] = {
                        "mean": np.mean(values),
                        "min": np.min(values),
                        "max": np.max(values),
                        "std": np.std(values)
                    }
                except:
                    pass
        
        return stats
    
    def clear_device_data(self, device_id: str) -> bool:
        """
        Clear the data history for a device
        
        Args:
            device_id: The ID of the device to clear data for
            
        Returns:
            True if successful, False if device not found
        """
        if device_id not in self.data_buffers:
            return False
        
        # Clear all buffers
        for buffer in self.data_buffers[device_id].values():
            buffer.clear()
        
        return True
