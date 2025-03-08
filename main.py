"""
Main Application Module for RoArm-M3 Isaac Sim Bridge

This module ties together all the components of the RoArm-M3 Isaac Sim Bridge
and provides the main entry point for the application.
"""

import time
import logging
import threading
import signal
import sys
import json
from typing import Dict, List, Optional

# Import configuration
import config

# Import modules
from src.device_discovery import DeviceManager
from src.data_processor import TelemetryProcessor, DeviceState
from src.camera_capture import CameraManager
from src.isaac_integration import IsaacSimBridge

class RoArmIsaacBridge:
    """
    Main class for the RoArm-M3 Isaac Sim Bridge
    """
    def __init__(self):
        """
        Initialize the RoArm-Isaac bridge
        """
        self.logger = logging.getLogger("RoArmIsaacBridge")
        self._setup_logging()
        
        self.device_manager = DeviceManager(config)
        self.data_processor = TelemetryProcessor(config)
        self.camera_manager = CameraManager(config)
        self.isaac_bridge = IsaacSimBridge(config)
        
        self.running = False
        self.threads = {}
    
    def _setup_logging(self):
        """
        Set up logging for the application
        """
        logging.basicConfig(
            level=getattr(logging, config.LOG_LEVEL),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            filename=config.LOG_FILE
        )
        
        # Also log to console
        console = logging.StreamHandler()
        console.setLevel(getattr(logging, config.LOG_LEVEL))
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console.setFormatter(formatter)
        logging.getLogger('').addHandler(console)
    
    def start(self):
        """
        Start the RoArm-Isaac bridge
        """
        self.logger.info("Starting RoArm-Isaac bridge")
        
        # Discover devices
        self.logger.info("Discovering RoArm-M3 devices...")
        devices = self.device_manager.discover_devices()
        
        if not devices:
            self.logger.warning("No RoArm-M3 devices found. Will continue monitoring for new devices.")
        else:
            self.logger.info(f"Found {len(devices)} RoArm-M3 devices")
        
        # Start device monitoring
        self.device_manager.monitor_devices()
        
        # Initialize camera
        self.logger.info("Initializing camera...")
        if not self.camera_manager.initialize_camera():
            self.logger.warning("Failed to initialize camera. Will continue without camera support.")
        else:
            self.logger.info("Camera initialized successfully")
            self.camera_manager.start_capture()
        
        # Connect to Isaac Sim
        self.logger.info("Connecting to Isaac Sim...")
        if not self.isaac_bridge.connect():
            self.logger.warning("Failed to connect to Isaac Sim. Will continue and try to reconnect later.")
        else:
            self.logger.info("Connected to Isaac Sim successfully")
            self.isaac_bridge.start_sending()
        
        # Start data collection threads for each device
        self.running = True
        for device_id, device_info in devices.items():
            self._start_device_thread(device_id, device_info)
        
        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.logger.info("RoArm-Isaac bridge started")
        
        # Main loop - monitor for new devices
        try:
            while self.running:
                # Check for new devices
                current_devices = self.device_manager.get_all_devices()
                for device_id, device_info in current_devices.items():
                    if device_id not in self.threads or not self.threads[device_id].is_alive():
                        self._start_device_thread(device_id, device_info)
                
                # Sleep to avoid busy waiting
                time.sleep(1)
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received, stopping...")
            self.stop()
    
    def _start_device_thread(self, device_id, device_info):
        """
        Start a data collection thread for a device
        
        Args:
            device_id: The ID of the device
            device_info: Information about the device
        """
        if device_id in self.threads and self.threads[device_id].is_alive():
            return
        
        self.threads[device_id] = threading.Thread(
            target=self._device_thread,
            args=(device_id, device_info),
            daemon=True
        )
        self.threads[device_id].start()
        
        self.logger.info(f"Started data collection thread for device {device_id}")
    
    def _device_thread(self, device_id, device_info):
        """
        Thread for collecting data from a device
        
        Args:
            device_id: The ID of the device
            device_info: Information about the device
        """
        serial_conn = device_info["serial"]
        failure_count = 0
        last_reset_time = 0
        
        while self.running and device_info["connected"]:
            try:
                # Check if there's data available
                if serial_conn.in_waiting:
                    # Read a line of data
                    line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Process the data
                    processed_data = self.data_processor.process_telemetry(device_id, line)
                    
                    if processed_data:
                        # Reset failure count on successful data processing
                        failure_count = 0
                        
                        # Get the latest camera frame
                        camera_data = self.camera_manager.get_latest_frame()
                        
                        # If we have a camera frame, add its timestamp to the data
                        if camera_data:
                            processed_data["camera_timestamp"] = camera_data["timestamp"]
                        
                        # Send the data to Isaac Sim
                        self.isaac_bridge.send_data(processed_data)
                        
                        # Handle device state if present
                        if "device_state" in processed_data:
                            device_state = DeviceState[processed_data["device_state"]]
                            self.handle_device_state(device_id, device_state, 
                                                   self.data_processor, self.isaac_bridge)
                    else:
                        # Increment failure count on data processing failure
                        failure_count += 1
                
                # Check if we should reset the device
                if failure_count > 0 and self.data_processor.should_reset_device(
                    device_id, failure_count, last_reset_time):
                    self.logger.warning(f"Resetting device {device_id} after {failure_count} failures")
                    
                    # Reset the device
                    self.device_manager.reset_device(device_id)
                    last_reset_time = time.time()
                    failure_count = 0
                
                # Sleep to avoid busy waiting
                time.sleep(0.001)
                
            except Exception as e:
                self.logger.error(f"Error in device thread for {device_id}: {str(e)}")
                failure_count += 1
                time.sleep(1)  # Sleep to avoid rapid error loops
    
    def _signal_handler(self, sig, frame):
        """
        Handle signals
        
        Args:
            sig: Signal number
            frame: Current stack frame
        """
        self.logger.info(f"Received signal {sig}, stopping...")
        self.stop()
        
    def handle_device_state(self, device_id, device_state, data_processor, isaac_bridge):
        """
        Handle different device states
        
        Args:
            device_id: The ID of the device
            device_state: The current state of the device
            data_processor: The data processor instance
            isaac_bridge: The Isaac Sim bridge instance
        """
        if device_state == DeviceState.NORMAL_OPERATION:
            # Normal operation, continue sending data to Isaac Sim
            pass
        elif device_state == DeviceState.UNPOWERED_SERVOS:
            # Unpowered servos, notify user but don't reset
            self.logger.warning(f"Device {device_id} has unpowered servos. Please check power supply.")
            # Continue sending data to Isaac Sim with warning flag
            isaac_bridge.send_status_update(device_id, "UNPOWERED_SERVOS", 
                                          "Device has unpowered servos. Please check power supply.")
        elif device_state == DeviceState.PARTIAL_OPERATION:
            # Partial operation, notify user and monitor
            self.logger.warning(f"Device {device_id} has partial operation. Some servos may not be working.")
            # Continue sending data to Isaac Sim with warning flag
            isaac_bridge.send_status_update(device_id, "PARTIAL_OPERATION", 
                                          "Device has partial operation. Some servos may not be working.")
        elif device_state == DeviceState.COMMUNICATION_FAILURE:
            # Communication failure, may need reset if persistent
            self.logger.error(f"Device {device_id} has communication failure.")
            # Send error status to Isaac Sim
            isaac_bridge.send_status_update(device_id, "COMMUNICATION_FAILURE", 
                                          "Device has communication failure.")
    
    def stop(self):
        """
        Stop the RoArm-Isaac bridge
        """
        self.logger.info("Stopping RoArm-Isaac bridge")
        
        self.running = False
        
        # Stop all threads
        for device_id, thread in self.threads.items():
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        # Close all devices
        self.device_manager.close_all()
        
        # Stop camera
        self.camera_manager.close_all()
        
        # Stop Isaac Sim bridge
        self.isaac_bridge.stop()
        
        self.logger.info("RoArm-Isaac bridge stopped")
        sys.exit(0)

if __name__ == "__main__":
    bridge = RoArmIsaacBridge()
    bridge.start()
