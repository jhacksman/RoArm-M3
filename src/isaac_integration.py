"""
Isaac Sim Integration Module for RoArm-M3 Isaac Sim Bridge

This module handles the communication between the RoArm-M3 Isaac Sim Bridge
and Isaac Sim. It provides a socket-based interface for sending telemetry
data to Isaac Sim.
"""

import json
import time
import logging
import socket
import threading
from typing import Dict, List, Optional

class IsaacSimBridge:
    """
    Handles communication with Isaac Sim
    """
    def __init__(self, config):
        """
        Initialize the IsaacSimBridge
        
        Args:
            config: Configuration object with settings
        """
        self.config = config
        self.logger = logging.getLogger("IsaacSimBridge")
        self.socket = None
        self.connected = False
        self.send_thread = None
        self.running = False
        self.data_queue = []
        self.lock = threading.Lock()
    
    def connect(self):
        """
        Connect to Isaac Sim
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.config.ISAAC_HOST, self.config.ISAAC_PORT))
            self.connected = True
            self.logger.info(f"Connected to Isaac Sim at {self.config.ISAAC_HOST}:{self.config.ISAAC_PORT}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to Isaac Sim: {str(e)}")
            self.connected = False
            return False
    
    def start_sending(self):
        """
        Start sending data to Isaac Sim
        
        Returns:
            True if successful, False otherwise
        """
        if not self.connected and not self.connect():
            return False
        
        if self.running:
            return True
        
        self.running = True
        self.send_thread = threading.Thread(
            target=self._send_data_thread,
            daemon=True
        )
        self.send_thread.start()
        
        self.logger.info("Started sending data to Isaac Sim")
        return True
    
    def _send_data_thread(self):
        """
        Thread for sending data to Isaac Sim
        """
        while self.running:
            try:
                # Get data from queue
                data_to_send = None
                with self.lock:
                    if self.data_queue:
                        data_to_send = self.data_queue.pop(0)
                
                if data_to_send:
                    # Convert data to JSON
                    json_data = json.dumps(data_to_send)
                    
                    # Send data
                    self.socket.sendall((json_data + "\n").encode())
                
                # Sleep to avoid busy waiting
                time.sleep(0.001)
                
            except Exception as e:
                self.logger.error(f"Error sending data to Isaac Sim: {str(e)}")
                self.connected = False
                
                # Try to reconnect
                for _ in range(5):  # Try 5 times
                    if self.connect():
                        break
                    time.sleep(1)
                
                if not self.connected:
                    self.logger.error("Failed to reconnect to Isaac Sim, stopping send thread")
                    self.running = False
                    break
    
    def send_data(self, data: Dict):
        """
        Queue data to be sent to Isaac Sim
        
        Args:
            data: Data dictionary to send
        """
        with self.lock:
            self.data_queue.append(data)
    
    def stop(self):
        """
        Stop sending data to Isaac Sim
        """
        self.running = False
        
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        self.connected = False
        self.logger.info("Stopped Isaac Sim bridge")
    
    def is_connected(self):
        """
        Check if the bridge is connected to Isaac Sim
        
        Returns:
            True if connected, False otherwise
        """
        return self.connected
    
    def get_queue_size(self):
        """
        Get the current size of the data queue
        
        Returns:
            Number of items in the data queue
        """
        with self.lock:
            return len(self.data_queue)
    
    def clear_queue(self):
        """
        Clear the data queue
        """
        with self.lock:
            self.data_queue.clear()
        
        self.logger.info("Cleared data queue")
