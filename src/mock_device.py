"""
Mock Device Module for RoArm-M3 Isaac Sim Bridge

This module simulates a RoArm-M3 device for testing purposes.
It generates mock telemetry data in the same format as the real device.
It also responds to serial commands like the real device.
"""

import json
import time
import math
import serial
import logging
import threading
import random
import queue
from typing import Dict, Optional

# Command definitions for RoArm-M3 devices
CMD_REBOOT = 600
CMD_GET_MAC_ADDRESS = 302

class MockRoArmM3Device:
    """
    Simulates a RoArm-M3 device for testing purposes
    """
    def __init__(self, mac_address="2C:BC:BB:4D:AC:C0", port=None):
        """
        Initialize the mock device
        
        Args:
            mac_address: MAC address to use for the device
            port: Serial port to use (if None, no serial output is generated)
        """
        self.mac_address = mac_address
        self.port = port
        self.serial = None
        
        self.logger = logging.getLogger("MockRoArmM3")
        self.running = False
        self.thread = None
        self.command_thread = None
        
        # Queue for incoming commands
        self.command_queue = queue.Queue()
        
        # Initial position
        self.x = 350.0
        self.y = 0.0
        self.z = 200.0
        
        # Initial joint angles
        self.b = 0.0  # Base angle (radians)
        self.s = 0.03  # Shoulder angle (radians)
        self.e = 1.6  # Elbow angle (radians)
        self.t = 0.01  # Wrist angle (radians)
        self.r = 0.0  # Roll angle (radians)
        self.g = 3.14  # Gripper angle (radians)
        
        # Joint loads/temperatures
        self.tB = 9
        self.tS = 89
        self.tE = 73
        self.tT = 33
        self.tR = 0
        self.tG = 28
        
        # Movement parameters
        self.time_offset = 0
        self.movement_radius = 50.0
        self.movement_speed = 0.5
    
    def connect(self):
        """
        Connect to the serial port if specified
        
        Returns:
            True if successful, False otherwise
        """
        if not self.port:
            return True
        
        try:
            self.serial = serial.Serial(self.port, 115200)
            return True
        except Exception as e:
            self.logger.error(f"Error connecting to serial port {self.port}: {str(e)}")
            return False
            
    def _send_initialization_messages(self):
        """
        Send the initialization messages that the real device would send
        """
        if self.serial:
            self.serial.write(b"All bus servos status checked.\r\n")
            time.sleep(0.1)
            self.serial.write(b"Server Starts.\r\n")
            time.sleep(0.1)
            self.serial.write(f"MAC Address: {self.mac_address}\r\n".encode())
            time.sleep(0.1)
        else:
            print("All bus servos status checked.")
            print("Server Starts.")
            print(f"MAC Address: {self.mac_address}")
    
    def _handle_commands(self):
        """
        Thread for handling incoming serial commands
        """
        self.logger.info("Started command handler thread")
        
        # Buffer for incoming data
        buffer = ""
        
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    # Read data from the serial port
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Try to parse as JSON
                        try:
                            command = json.loads(line)
                            self.logger.debug(f"Received command: {command}")
                            
                            # Handle the command
                            if "T" in command:
                                cmd_type = command["T"]
                                
                                if cmd_type == CMD_REBOOT:
                                    self.logger.info("Received reboot command")
                                    # Simulate a reboot
                                    self.command_queue.put(("reboot", None))
                                
                                elif cmd_type == CMD_GET_MAC_ADDRESS:
                                    self.logger.info("Received get MAC address command")
                                    # Send the MAC address
                                    if self.serial:
                                        self.serial.write(f"{self.mac_address}\r\n".encode())
                        
                        except json.JSONDecodeError:
                            # Not a valid JSON command, ignore
                            pass
                
                # Check for commands from the queue
                try:
                    cmd, data = self.command_queue.get(block=False)
                    
                    if cmd == "reboot":
                        self.logger.info("Executing reboot command")
                        # Stop sending telemetry
                        if self.thread and self.thread.is_alive():
                            self.thread.join(timeout=0.5)
                        
                        # Send initialization messages
                        self._send_initialization_messages()
                        
                        # Restart telemetry thread
                        self.thread = threading.Thread(target=self._telemetry_thread, daemon=True)
                        self.thread.start()
                        
                except queue.Empty:
                    pass
                
                # Sleep to avoid busy waiting
                time.sleep(0.01)
                
            except Exception as e:
                self.logger.error(f"Error in command handler: {str(e)}")
                time.sleep(0.1)
    
    def start(self):
        """
        Start generating telemetry data and command handling
        
        Returns:
            True if successful, False otherwise
        """
        if self.running:
            return True
        
        if self.port and not self.serial:
            if not self.connect():
                return False
        
        self.running = True
        
        # Start the telemetry thread
        self.thread = threading.Thread(target=self._telemetry_thread, daemon=True)
        self.thread.start()
        
        # Start the command handler thread
        self.command_thread = threading.Thread(target=self._handle_commands, daemon=True)
        self.command_thread.start()
        
        self.logger.info(f"Started mock RoArm-M3 device with MAC {self.mac_address}")
        return True
    
    def _telemetry_thread(self):
        """
        Thread for generating telemetry data
        """
        # Print initialization messages
        self._send_initialization_messages()
        
        start_time = time.time()
        
        while self.running:
            try:
                # Calculate elapsed time
                elapsed = time.time() - start_time + self.time_offset
                
                # Update position with circular movement
                self.x = 350.0 + self.movement_radius * math.cos(elapsed * self.movement_speed)
                self.y = self.movement_radius * math.sin(elapsed * self.movement_speed)
                
                # Add some noise to the position
                self.x += random.uniform(-1.0, 1.0)
                self.y += random.uniform(-1.0, 1.0)
                self.z += random.uniform(-0.5, 0.5)
                
                # Update joint angles based on position
                self.b = math.atan2(self.y, self.x)
                
                # Create telemetry data
                telemetry = {
                    "T": 1051,
                    "x": self.x,
                    "y": self.y,
                    "z": self.z,
                    "tit": 0.065961174,
                    "b": self.b,
                    "s": self.s,
                    "e": self.e,
                    "t": self.t,
                    "r": self.r,
                    "g": self.g,
                    "tB": self.tB,
                    "tS": self.tS,
                    "tE": self.tE,
                    "tT": self.tT,
                    "tR": self.tR,
                    "tG": self.tG,
                    "mac": self.mac_address
                }
                
                # Convert to JSON
                json_data = json.dumps(telemetry)
                
                # Send telemetry data
                if self.serial and self.serial.is_open:
                    try:
                        self.serial.write((json_data + "\r\n").encode())
                    except Exception as e:
                        self.logger.error(f"Error writing telemetry data: {str(e)}")
                else:
                    print(json_data)
                
                # Sleep to maintain 100Hz rate
                time.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error in telemetry thread: {str(e)}")
                time.sleep(0.1)  # Sleep longer on error
    
    def stop(self):
        """
        Stop generating telemetry data and command handling
        """
        self.running = False
        
        # Stop the telemetry thread
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        
        # Stop the command handler thread
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
        
        # Close the serial connection
        if self.serial:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception as e:
                self.logger.error(f"Error closing serial connection: {str(e)}")
            self.serial = None
        
        self.logger.info("Stopped mock RoArm-M3 device")

if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create a mock device
    device = MockRoArmM3Device()
    
    try:
        # Start the device
        device.start()
        
        # Run until interrupted
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Stop the device
        device.stop()
