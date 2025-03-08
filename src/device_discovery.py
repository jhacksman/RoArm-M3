"""
Device Discovery Module for RoArm-M3 Isaac Sim Bridge

This module handles the discovery and management of RoArm-M3 devices connected to the system.
It provides auto-provisioning capabilities for plug-and-play compatibility.
"""

import serial
import serial.tools.list_ports
import json
import time
import logging
import threading
import re
from typing import Dict, List, Optional, Tuple

# Command definitions for RoArm-M3 devices
CMD_REBOOT = 600
CMD_GET_MAC_ADDRESS = 302

# Patterns for parsing device output
MAC_ADDRESS_PATTERN = re.compile(r'MAC Address: ([0-9A-F:]{17})')
INIT_COMPLETE_MESSAGES = [
    "All bus servos status checked.",
    "Server Starts."
]

try:
    import pyudev
    PYUDEV_AVAILABLE = True
except ImportError:
    PYUDEV_AVAILABLE = False
    print("pyudev not available, hot-plugging detection will be disabled")

class DeviceManager:
    """
    Manages the discovery and connection to RoArm-M3 devices
    """
    def __init__(self, config):
        """
        Initialize the DeviceManager
        
        Args:
            config: Configuration object with settings
        """
        self.config = config
        self.devices = {}  # Dictionary to store connected devices
        self.logger = logging.getLogger("DeviceManager")
        self._setup_logging()
        
        # Set up pyudev for device monitoring if available
        self.monitor = None
        self.observer = None
        if PYUDEV_AVAILABLE:
            try:
                self.context = pyudev.Context()
                self.monitor = pyudev.Monitor.from_netlink(self.context)
                self.monitor.filter_by(subsystem='tty')
            except Exception as e:
                self.logger.error(f"Error setting up pyudev: {str(e)}")
        
        # Lock for thread-safe access to devices dictionary
        self.devices_lock = threading.Lock()
        
    def reset_device(self, serial_port):
        """
        Reset a RoArm-M3 device by sending the reboot command
        
        Args:
            serial_port: Serial port object connected to the device
            
        Returns:
            bool: True if the reset command was sent successfully, False otherwise
        """
        try:
            # Send the reboot command
            reset_cmd = json.dumps({"T": CMD_REBOOT}) + "\n"
            serial_port.write(reset_cmd.encode())
            self.logger.info(f"Sent reset command to device on {serial_port.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to reset device on {serial_port.port}: {str(e)}")
            return False
    
    def wait_for_device_init(self, serial_port, timeout=10):
        """
        Wait for a device to complete initialization after reset
        
        Args:
            serial_port: Serial port object connected to the device
            timeout: Maximum time to wait in seconds
            
        Returns:
            str: MAC address if found, None otherwise
        """
        start_time = time.time()
        init_messages_seen = set()
        mac_address = None
        
        self.logger.info(f"Waiting for device initialization on {serial_port.port}")
        
        while time.time() - start_time < timeout:
            try:
                if serial_port.in_waiting:
                    line = serial_port.readline().decode('utf-8', errors='ignore').strip()
                    self.logger.debug(f"Received: {line}")
                    
                    # Check for initialization messages
                    for msg in INIT_COMPLETE_MESSAGES:
                        if msg in line:
                            init_messages_seen.add(msg)
                            self.logger.info(f"Detected initialization message: {msg}")
                    
                    # Check for MAC address
                    match = MAC_ADDRESS_PATTERN.search(line)
                    if match:
                        mac_address = match.group(1)
                        self.logger.info(f"Found MAC address: {mac_address}")
                    
                    # If we have all init messages and the MAC address, we're done
                    if len(init_messages_seen) == len(INIT_COMPLETE_MESSAGES) and mac_address:
                        return mac_address
            except Exception as e:
                self.logger.error(f"Error reading from {serial_port.port}: {str(e)}")
                break
                
            # Sleep to avoid busy waiting
            time.sleep(0.01)
        
        self.logger.warning(f"Timeout waiting for device initialization on {serial_port.port}")
        return mac_address
    
    def request_mac_address(self, serial_port, timeout=5):
        """
        Request the MAC address from a device using the GET_MAC_ADDRESS command
        
        Args:
            serial_port: Serial port object connected to the device
            timeout: Maximum time to wait in seconds
            
        Returns:
            str: MAC address if found, None otherwise
        """
        try:
            # Send the get MAC address command
            cmd = json.dumps({"T": CMD_GET_MAC_ADDRESS}) + "\n"
            serial_port.write(cmd.encode())
            self.logger.info(f"Sent get MAC address command to device on {serial_port.port}")
            
            # Wait for the response
            start_time = time.time()
            while time.time() - start_time < timeout:
                if serial_port.in_waiting:
                    line = serial_port.readline().decode('utf-8', errors='ignore').strip()
                    self.logger.debug(f"Received: {line}")
                    
                    # Check if the line is a MAC address (format: XX:XX:XX:XX:XX:XX)
                    if re.match(r'^([0-9A-F]{2}:){5}[0-9A-F]{2}$', line):
                        self.logger.info(f"Found MAC address: {line}")
                        return line
                
                # Sleep to avoid busy waiting
                time.sleep(0.01)
            
            self.logger.warning(f"Timeout waiting for MAC address on {serial_port.port}")
            return None
        except Exception as e:
            self.logger.error(f"Failed to get MAC address from device on {serial_port.port}: {str(e)}")
            return None
    
    def _setup_logging(self):
        """
        Set up logging for the DeviceManager
        """
        logging.basicConfig(
            level=getattr(logging, self.config.LOG_LEVEL),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            filename=self.config.LOG_FILE
        )
        
        # Also log to console
        console = logging.StreamHandler()
        console.setLevel(getattr(logging, self.config.LOG_LEVEL))
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console.setFormatter(formatter)
        logging.getLogger('').addHandler(console)
    
    def _get_available_ports(self):
        """
        Get a list of available serial ports
        
        Returns:
            List of available serial ports
        """
        return serial.tools.list_ports.comports()
    
    def discover_devices(self) -> Dict[str, Dict]:
        """
        Discover RoArm-M3 devices connected to the system
        
        Returns:
            dict: Dictionary of discovered devices, keyed by device ID
        """
        self.logger.info("Discovering RoArm-M3 devices...")
        
        # Get a list of available serial ports
        available_ports = self._get_available_ports()
        self.logger.info(f"Found {len(available_ports)} serial ports")
        
        discovered_devices = {}
        
        for port_info in available_ports:
            try:
                self.logger.info(f"Checking port {port_info.device}")
                
                # Skip if we already have this port
                with self.devices_lock:
                    if any(d["port"] == port_info.device and d["connected"] for d in self.devices.values()):
                        self.logger.info(f"Port {port_info.device} is already connected to a device, skipping")
                        continue
                
                # Try to open the port
                serial_conn = serial.Serial(
                    port=port_info.device,
                    baudrate=self.config.SERIAL_BAUDRATE,
                    timeout=self.config.SERIAL_TIMEOUT
                )
                
                # First, try to get the MAC address by requesting it
                mac_address = self.request_mac_address(serial_conn)
                
                # If that fails, try resetting the device and capturing the MAC address during initialization
                if not mac_address:
                    self.logger.info(f"Resetting device on {port_info.device} to get MAC address")
                    if self.reset_device(serial_conn):
                        mac_address = self.wait_for_device_init(serial_conn)
                
                # If we found a MAC address, register the device
                if mac_address:
                    device_id = mac_address if self.config.USE_MAC_ADDRESS else port_info.device
                    self.logger.info(f"Discovered RoArm-M3 device with MAC {mac_address} on {port_info.device}")
                    
                    # Register the device
                    discovered_devices[device_id] = {
                        "id": device_id,
                        "type": "RoArm-M3",
                        "port": port_info.device,
                        "mac_address": mac_address,
                        "serial": serial_conn,
                        "connected": True,
                        "last_seen": time.time()
                    }
                else:
                    # Not a RoArm-M3 device or couldn't get MAC address
                    self.logger.info(f"No RoArm-M3 device found on {port_info.device}")
                    serial_conn.close()
                    
            except Exception as e:
                self.logger.error(f"Error checking port {port_info.device}: {str(e)}")
                continue
        
        # Update the devices dictionary
        with self.devices_lock:
            for device_id, device_info in discovered_devices.items():
                self.devices[device_id] = device_info
        
        return self.get_all_devices()
    
    def monitor_devices(self):
        """
        Start monitoring for device connections/disconnections
        """
        self.monitoring = True
        
        if not PYUDEV_AVAILABLE or not self.monitor:
            self.logger.warning("pyudev not available, using polling for device detection")
            # Start a polling thread instead
            self.polling_thread = threading.Thread(
                target=self._poll_devices,
                daemon=True
            )
            self.polling_thread.start()
            return
        
        try:
            self.observer = pyudev.MonitorObserver(self.monitor, self._device_event)
            self.observer.start()
            self.logger.info("Started device monitoring with pyudev")
        except Exception as e:
            self.logger.error(f"Error starting device monitoring: {str(e)}")
            # Fall back to polling
            self.polling_thread = threading.Thread(
                target=self._poll_devices,
                daemon=True
            )
            self.polling_thread.start()
    
    def _poll_devices(self):
        """
        Thread for monitoring device connections/disconnections
        Used as a fallback when pyudev is not available
        """
        self.logger.info("Started device polling thread")
        
        while self.monitoring:
            try:
                # Check for new devices
                available_ports = self._get_available_ports()
                available_port_paths = [p.device for p in available_ports]
                
                # Check for disconnected devices
                with self.devices_lock:
                    for device_id, device_info in list(self.devices.items()):
                        if device_info["connected"] and device_info["port"] not in available_port_paths:
                            self.logger.info(f"Device {device_id} disconnected")
                            device_info["connected"] = False
                            if device_info["serial"] and device_info["serial"].is_open:
                                try:
                                    device_info["serial"].close()
                                except:
                                    pass
                
                # Check for new or reconnected devices
                for port_info in available_ports:
                    # Skip ports that are already connected to a device
                    skip = False
                    with self.devices_lock:
                        for device_info in self.devices.values():
                            if device_info["connected"] and device_info["port"] == port_info.device:
                                skip = True
                                break
                    
                    if skip:
                        continue
                    
                    # Check if this port belongs to a previously disconnected device
                    reconnected = False
                    with self.devices_lock:
                        for device_id, device_info in self.devices.items():
                            if not device_info["connected"] and device_info["port"] == port_info.device:
                                try:
                                    # Try to reconnect
                                    serial_conn = serial.Serial(
                                        port=port_info.device,
                                        baudrate=self.config.SERIAL_BAUDRATE,
                                        timeout=self.config.SERIAL_TIMEOUT
                                    )
                                    
                                    # Reset the device to get the MAC address
                                    if self.reset_device(serial_conn):
                                        mac_address = self.wait_for_device_init(serial_conn)
                                        
                                        if mac_address == device_info["mac_address"]:
                                            # This is the same device
                                            self.logger.info(f"Device {device_id} reconnected")
                                            device_info["connected"] = True
                                            device_info["serial"] = serial_conn
                                            device_info["last_seen"] = time.time()
                                            reconnected = True
                                            break
                                        else:
                                            # Different device
                                            serial_conn.close()
                                except Exception as e:
                                    self.logger.error(f"Error reconnecting to device on {port_info.device}: {str(e)}")
                    
                    if not reconnected:
                        # This might be a new device, try to discover it
                        try:
                            serial_conn = serial.Serial(
                                port=port_info.device,
                                baudrate=self.config.SERIAL_BAUDRATE,
                                timeout=self.config.SERIAL_TIMEOUT
                            )
                            
                            # Reset the device to get the MAC address
                            if self.reset_device(serial_conn):
                                mac_address = self.wait_for_device_init(serial_conn)
                                
                                if mac_address:
                                    # Check if we already know this device by MAC address
                                    known_device = False
                                    with self.devices_lock:
                                        for device_id, device_info in self.devices.items():
                                            if device_info["mac_address"] == mac_address:
                                                # This is a known device on a new port
                                                self.logger.info(f"Device {device_id} reconnected on new port {port_info.device}")
                                                device_info["connected"] = True
                                                device_info["port"] = port_info.device
                                                device_info["serial"] = serial_conn
                                                device_info["last_seen"] = time.time()
                                                known_device = True
                                                break
                                    
                                    if not known_device:
                                        # This is a new device
                                        device_id = mac_address
                                        self.logger.info(f"Discovered new RoArm-M3 device with MAC {mac_address} on {port_info.device}")
                                        
                                        with self.devices_lock:
                                            self.devices[device_id] = {
                                                "id": device_id,
                                                "type": "RoArm-M3",
                                                "port": port_info.device,
                                                "mac_address": mac_address,
                                                "serial": serial_conn,
                                                "connected": True,
                                                "last_seen": time.time()
                                            }
                                else:
                                    # Not a RoArm-M3 device or couldn't get MAC address
                                    serial_conn.close()
                        except Exception as e:
                            self.logger.error(f"Error checking for new device on {port_info.device}: {str(e)}")
                
                # Sleep to avoid busy waiting
                time.sleep(1)
            except Exception as e:
                self.logger.error(f"Error in device monitor thread: {str(e)}")
                time.sleep(5)  # Sleep longer on error
    
    def _device_event(self, action, device):
        """
        Handle device connection/disconnection events from pyudev
        
        Args:
            action: The action that occurred (add, remove, etc.)
            device: The device that was affected
        """
        if action == 'add':
            device_name = device.get('DEVNAME')
            if not device_name:
                return
                
            self.logger.info(f"New device connected: {device_name}")
            # Wait a moment for the device to initialize
            time.sleep(2)
            
            try:
                # Try to open the port
                serial_conn = serial.Serial(
                    port=device_name,
                    baudrate=self.config.SERIAL_BAUDRATE,
                    timeout=self.config.SERIAL_TIMEOUT
                )
                
                # First, try to get the MAC address by requesting it
                mac_address = self.request_mac_address(serial_conn)
                
                # If that fails, try resetting the device and capturing the MAC address during initialization
                if not mac_address:
                    self.logger.info(f"Resetting device on {device_name} to get MAC address")
                    if self.reset_device(serial_conn):
                        mac_address = self.wait_for_device_init(serial_conn)
                
                # If we found a MAC address, register the device
                if mac_address:
                    device_id = mac_address if self.config.USE_MAC_ADDRESS else device_name
                    self.logger.info(f"Discovered RoArm-M3 device with MAC {mac_address} on {device_name}")
                    
                    # Check if we already know this device by MAC address
                    known_device = False
                    with self.devices_lock:
                        for existing_id, device_info in self.devices.items():
                            if device_info["mac_address"] == mac_address:
                                # This is a known device on a new port
                                self.logger.info(f"Device {existing_id} reconnected on new port {device_name}")
                                device_info["connected"] = True
                                device_info["port"] = device_name
                                device_info["serial"] = serial_conn
                                device_info["last_seen"] = time.time()
                                known_device = True
                                break
                    
                    if not known_device:
                        # This is a new device
                        with self.devices_lock:
                            self.devices[device_id] = {
                                "id": device_id,
                                "type": "RoArm-M3",
                                "port": device_name,
                                "mac_address": mac_address,
                                "serial": serial_conn,
                                "connected": True,
                                "last_seen": time.time()
                            }
                else:
                    # Not a RoArm-M3 device or couldn't get MAC address
                    serial_conn.close()
            except Exception as e:
                self.logger.error(f"Error checking new device on {device_name}: {str(e)}")
        
        elif action == 'remove':
            device_name = device.get('DEVNAME')
            if not device_name:
                return
                
            self.logger.info(f"Device disconnected: {device_name}")
            
            # Find and mark the device as disconnected
            with self.devices_lock:
                for device_id, device_info in self.devices.items():
                    if device_info["port"] == device_name:
                        device_info["connected"] = False
                        if device_info["serial"] and device_info["serial"].is_open:
                            try:
                                device_info["serial"].close()
                            except:
                                pass
                        self.logger.info(f"Marked device {device_id} as disconnected")
    
    def get_device_by_id(self, device_id: str) -> Optional[Dict]:
        """
        Get device info by device ID
        
        Args:
            device_id: The ID of the device to get
            
        Returns:
            The device info dictionary or None if not found
        """
        return self.devices.get(device_id)
    
    def get_all_devices(self) -> Dict[str, Dict]:
        """
        Get all connected devices
        
        Returns:
            A dictionary of connected devices
        """
        return {k: v for k, v in self.devices.items() if v["connected"]}
    
    def close_all(self):
        """
        Close all serial connections and stop monitoring
        """
        # Stop monitoring
        self.monitoring = False
        
        # Close all serial connections
        with self.devices_lock:
            for device_id, device_info in self.devices.items():
                if "serial" in device_info and device_info["serial"] and device_info["serial"].is_open:
                    try:
                        device_info["serial"].close()
                    except Exception as e:
                        self.logger.error(f"Error closing serial connection for device {device_id}: {str(e)}")
        
        # Stop the pyudev observer if it's running
        if hasattr(self, 'observer') and self.observer and self.observer.is_alive():
            try:
                self.observer.stop()
            except Exception as e:
                self.logger.error(f"Error stopping pyudev observer: {str(e)}")
        
        # The polling thread is a daemon thread and will be terminated when the program exits
        # We've set self.monitoring = False which should cause it to exit its loop
        
        self.logger.info("Closed all device connections and stopped monitoring")
