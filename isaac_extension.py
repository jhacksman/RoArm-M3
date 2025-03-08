"""
Isaac Sim Extension Module for RoArm-M3 Isaac Sim Bridge

This module provides an extension for Isaac Sim that receives telemetry data
from the RoArm-M3 Isaac Sim Bridge and visualizes it in Isaac Sim.
"""

import socket
import threading
import json
import time
import logging
import os
from typing import Dict, List, Optional

import omni.ext
import omni.ui as ui
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat

class RoArmBridgeExtension(omni.ext.IExt):
    """
    Isaac Sim extension for visualizing RoArm-M3 telemetry data
    """
    def __init__(self):
        """
        Initialize the RoArmBridgeExtension
        """
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self._setup_logging()
        
        self.server_socket = None
        self.client_socket = None
        self.server_thread = None
        self.running = False
        
        self.devices = {}  # Dictionary to store connected devices
        self.world = None
        self.ui_window = None
        
        # Settings
        self.host = "0.0.0.0"  # Listen on all interfaces
        self.port = 8000
        self.update_rate = 100  # Hz
    
    def _setup_logging(self):
        """
        Set up logging for the extension
        """
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            filename=os.path.join(os.path.expanduser("~"), "roarm_isaac_bridge", "isaac_extension.log")
        )
        
        # Also log to console
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console.setFormatter(formatter)
        logging.getLogger('').addHandler(console)
    
    def on_startup(self, ext_id):
        """
        Called when the extension is started
        
        Args:
            ext_id: Extension ID
        """
        self.logger.info(f"Starting RoArm Bridge Extension {ext_id}")
        
        # Initialize the world
        self.world = World()
        
        # Create UI
        self._build_ui()
        
        # Start the server
        self.start_server()
    
    def on_shutdown(self):
        """
        Called when the extension is shut down
        """
        self.logger.info("Shutting down RoArm Bridge Extension")
        
        # Stop the server
        self.stop_server()
        
        # Clean up UI
        if self.ui_window:
            self.ui_window = None
    
    def _build_ui(self):
        """
        Build the UI for the extension
        """
        self.ui_window = ui.Window("RoArm-M3 Bridge", width=300, height=400)
        with self.ui_window.frame:
            with ui.VStack(spacing=5):
                ui.Label("RoArm-M3 Bridge Extension", height=30, style={"font_size": 20})
                
                with ui.HStack(height=30):
                    ui.Label("Server Status:", width=100)
                    self.status_label = ui.Label("Stopped", width=100)
                
                with ui.HStack(height=30):
                    ui.Label("Host:", width=100)
                    self.host_field = ui.StringField(width=150)
                    self.host_field.model.set_value(self.host)
                
                with ui.HStack(height=30):
                    ui.Label("Port:", width=100)
                    self.port_field = ui.IntField(width=150)
                    self.port_field.model.set_value(self.port)
                
                with ui.HStack(height=30):
                    ui.Label("Update Rate:", width=100)
                    self.rate_field = ui.IntField(width=150)
                    self.rate_field.model.set_value(self.update_rate)
                
                with ui.HStack(height=30):
                    self.start_button = ui.Button("Start Server", width=100)
                    self.start_button.set_clicked_fn(self._on_start_clicked)
                    
                    self.stop_button = ui.Button("Stop Server", width=100)
                    self.stop_button.set_clicked_fn(self._on_stop_clicked)
                
                ui.Spacer(height=10)
                ui.Label("Connected Devices:", height=30)
                
                self.device_list = ui.VStack(spacing=5)
                with self.device_list:
                    ui.Label("No devices connected", height=30)
    
    def _on_start_clicked(self):
        """
        Called when the Start Server button is clicked
        """
        self.host = self.host_field.model.get_value_as_string()
        self.port = self.port_field.model.get_value_as_int()
        self.update_rate = self.rate_field.model.get_value_as_int()
        
        self.start_server()
    
    def _on_stop_clicked(self):
        """
        Called when the Stop Server button is clicked
        """
        self.stop_server()
    
    def start_server(self):
        """
        Start the server to receive telemetry data
        """
        if self.running:
            return
        
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            
            self.running = True
            self.server_thread = threading.Thread(target=self._server_thread, daemon=True)
            self.server_thread.start()
            
            self.status_label.text = "Running"
            self.logger.info(f"Server started on {self.host}:{self.port}")
            
            return True
        except Exception as e:
            self.logger.error(f"Error starting server: {str(e)}")
            self.status_label.text = f"Error: {str(e)}"
            return False
    
    def stop_server(self):
        """
        Stop the server
        """
        if not self.running:
            return
        
        self.running = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            self.server_socket = None
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
        
        self.status_label.text = "Stopped"
        self.logger.info("Server stopped")
    
    def _server_thread(self):
        """
        Thread for handling server connections
        """
        self.logger.info("Server thread started")
        
        while self.running:
            try:
                # Accept a connection
                self.server_socket.settimeout(1.0)
                client_socket, client_address = self.server_socket.accept()
                self.client_socket = client_socket
                
                self.logger.info(f"Client connected from {client_address}")
                
                # Handle the connection
                self._handle_client(client_socket)
                
            except socket.timeout:
                # Timeout is expected, just continue
                pass
            except Exception as e:
                if self.running:
                    self.logger.error(f"Error in server thread: {str(e)}")
                    time.sleep(1.0)
        
        self.logger.info("Server thread stopped")
    
    def _handle_client(self, client_socket):
        """
        Handle a client connection
        
        Args:
            client_socket: Client socket
        """
        buffer = ""
        
        while self.running:
            try:
                # Receive data
                data = client_socket.recv(4096).decode('utf-8')
                
                if not data:
                    # Connection closed
                    break
                
                # Add data to buffer
                buffer += data
                
                # Process complete JSON objects
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    try:
                        # Parse JSON
                        telemetry = json.loads(line)
                        
                        # Process telemetry data
                        self._process_telemetry(telemetry)
                    except json.JSONDecodeError:
                        self.logger.warning(f"Invalid JSON: {line}")
                    except Exception as e:
                        self.logger.error(f"Error processing telemetry: {str(e)}")
                
            except Exception as e:
                if self.running:
                    self.logger.error(f"Error handling client: {str(e)}")
                break
        
        # Close the connection
        try:
            client_socket.close()
        except:
            pass
        
        self.logger.info("Client disconnected")
    
    def _process_telemetry(self, telemetry):
        """
        Process telemetry data from a device
        
        Args:
            telemetry: Telemetry data dictionary
        """
        # Extract device ID
        device_id = telemetry.get("device_id")
        
        if not device_id:
            self.logger.warning("Telemetry data missing device_id")
            return
        
        # Check if this is a new device
        if device_id not in self.devices:
            # Create a new device representation
            self._create_device(device_id)
        
        # Update the device representation
        self._update_device(device_id, telemetry)
    
    def _create_device(self, device_id):
        """
        Create a new device representation in Isaac Sim
        
        Args:
            device_id: Device ID
        """
        self.logger.info(f"Creating representation for device {device_id}")
        
        # Create a new device entry
        self.devices[device_id] = {
            "id": device_id,
            "last_update": time.time(),
            "arm_parts": {}
        }
        
        # Create the arm representation
        try:
            # Create base
            base = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/RoArm_{device_id}/base",
                    name=f"base_{device_id}",
                    position=[0, 0, 0],
                    scale=[0.1, 0.1, 0.05],
                    color=[0.8, 0.2, 0.2]
                )
            )
            self.devices[device_id]["arm_parts"]["base"] = base
            
            # Create shoulder
            shoulder = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/RoArm_{device_id}/shoulder",
                    name=f"shoulder_{device_id}",
                    position=[0, 0, 0.05],
                    scale=[0.05, 0.05, 0.15],
                    color=[0.2, 0.8, 0.2]
                )
            )
            self.devices[device_id]["arm_parts"]["shoulder"] = shoulder
            
            # Create elbow
            elbow = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/RoArm_{device_id}/elbow",
                    name=f"elbow_{device_id}",
                    position=[0, 0, 0.2],
                    scale=[0.05, 0.05, 0.15],
                    color=[0.2, 0.2, 0.8]
                )
            )
            self.devices[device_id]["arm_parts"]["elbow"] = elbow
            
            # Create wrist
            wrist = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/RoArm_{device_id}/wrist",
                    name=f"wrist_{device_id}",
                    position=[0, 0, 0.35],
                    scale=[0.03, 0.03, 0.05],
                    color=[0.8, 0.8, 0.2]
                )
            )
            self.devices[device_id]["arm_parts"]["wrist"] = wrist
            
            # Create gripper
            gripper = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/RoArm_{device_id}/gripper",
                    name=f"gripper_{device_id}",
                    position=[0, 0, 0.4],
                    scale=[0.02, 0.04, 0.02],
                    color=[0.8, 0.2, 0.8]
                )
            )
            self.devices[device_id]["arm_parts"]["gripper"] = gripper
            
            # Update the UI
            self._update_device_list()
            
        except Exception as e:
            self.logger.error(f"Error creating device representation: {str(e)}")
    
    def _update_device(self, device_id, telemetry):
        """
        Update a device representation with new telemetry data
        
        Args:
            device_id: Device ID
            telemetry: Telemetry data dictionary
        """
        if device_id not in self.devices:
            return
        
        device = self.devices[device_id]
        device["last_update"] = time.time()
        
        try:
            # Extract position and rotation data
            x = telemetry.get("x", 0) / 1000.0  # Convert mm to m
            y = telemetry.get("y", 0) / 1000.0
            z = telemetry.get("z", 0) / 1000.0
            
            b = telemetry.get("b", 0)  # Base angle (radians)
            s = telemetry.get("s", 0)  # Shoulder angle (radians)
            e = telemetry.get("e", 0)  # Elbow angle (radians)
            t = telemetry.get("t", 0)  # Wrist angle (radians)
            r = telemetry.get("r", 0)  # Roll angle (radians)
            g = telemetry.get("g", 0)  # Gripper angle (radians)
            
            # Update base position and rotation
            base_quat = euler_angles_to_quat([0, 0, b])
            device["arm_parts"]["base"].set_world_pose([0, 0, 0], base_quat)
            
            # Update shoulder position and rotation
            shoulder_quat = euler_angles_to_quat([0, s, 0])
            device["arm_parts"]["shoulder"].set_world_pose([0, 0, 0.05], shoulder_quat)
            
            # Update elbow position and rotation
            elbow_quat = euler_angles_to_quat([0, e, 0])
            device["arm_parts"]["elbow"].set_world_pose([x/2, y/2, 0.2], elbow_quat)
            
            # Update wrist position and rotation
            wrist_quat = euler_angles_to_quat([r, t, 0])
            device["arm_parts"]["wrist"].set_world_pose([x*0.8, y*0.8, 0.35], wrist_quat)
            
            # Update gripper position
            # Adjust gripper width based on gripper angle
            gripper_width = 0.04 * (1.0 - g / 3.14)
            device["arm_parts"]["gripper"].set_world_pose([x, y, z], [0, 0, 0, 1])
            device["arm_parts"]["gripper"].set_world_scale([0.02, gripper_width, 0.02])
            
        except Exception as e:
            self.logger.error(f"Error updating device representation: {str(e)}")
    
    def _update_device_list(self):
        """
        Update the device list in the UI
        """
        # Clear the device list
        for child in self.device_list.children:
            child.destroy()
        
        # Add devices to the list
        if not self.devices:
            with self.device_list:
                ui.Label("No devices connected", height=30)
        else:
            for device_id, device in self.devices.items():
                with self.device_list:
                    with ui.HStack(height=30):
                        ui.Label(f"Device: {device_id}", width=200)
                        ui.Label(f"Last update: {time.time() - device['last_update']:.1f}s ago", width=100)
