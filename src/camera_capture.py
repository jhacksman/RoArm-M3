"""
Camera Capture Module for RoArm-M3 Isaac Sim Bridge

This module handles camera initialization, frame capture, and synchronization
with telemetry data from RoArm-M3 devices.
"""

import cv2
import time
import logging
import threading
from typing import Dict, Optional
from queue import Queue

class CameraManager:
    """
    Manages camera initialization, frame capture, and synchronization
    """
    def __init__(self, config):
        """
        Initialize the CameraManager
        
        Args:
            config: Configuration object with settings
        """
        self.config = config
        self.logger = logging.getLogger("CameraManager")
        self.cameras = {}  # Dictionary to store camera objects
        self.frame_queues = {}  # Dictionary to store frame queues
        self.running = False
    
    def initialize_camera(self, camera_id=None):
        """
        Initialize a camera with the given ID or the default camera
        
        Args:
            camera_id: ID of the camera to initialize (default: None, uses config.CAMERA_INDEX)
            
        Returns:
            True if successful, False otherwise
        """
        camera_id = camera_id if camera_id is not None else self.config.CAMERA_INDEX
        
        try:
            # Initialize camera
            cap = cv2.VideoCapture(camera_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
            
            if not cap.isOpened():
                self.logger.error(f"Failed to open camera {camera_id}")
                return False
            
            self.cameras[camera_id] = {
                "capture": cap,
                "thread": None,
                "running": False
            }
            
            self.frame_queues[camera_id] = Queue(maxsize=100)
            
            self.logger.info(f"Initialized camera {camera_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing camera {camera_id}: {str(e)}")
            return False
    
    def start_capture(self, camera_id=None):
        """
        Start capturing frames from the camera
        
        Args:
            camera_id: ID of the camera to start capturing from (default: None, uses config.CAMERA_INDEX)
            
        Returns:
            True if successful, False otherwise
        """
        camera_id = camera_id if camera_id is not None else self.config.CAMERA_INDEX
        
        if camera_id not in self.cameras:
            if not self.initialize_camera(camera_id):
                return False
        
        camera_info = self.cameras[camera_id]
        
        if camera_info["running"]:
            return True
        
        camera_info["running"] = True
        camera_info["thread"] = threading.Thread(
            target=self._capture_frames,
            args=(camera_id,),
            daemon=True
        )
        camera_info["thread"].start()
        
        self.logger.info(f"Started capture on camera {camera_id}")
        return True
    
    def _capture_frames(self, camera_id):
        """
        Continuously capture frames from the camera
        
        Args:
            camera_id: ID of the camera to capture frames from
        """
        camera_info = self.cameras[camera_id]
        cap = camera_info["capture"]
        queue = self.frame_queues[camera_id]
        
        while camera_info["running"]:
            ret, frame = cap.read()
            
            if not ret:
                self.logger.warning(f"Failed to read frame from camera {camera_id}")
                time.sleep(0.01)
                continue
            
            # Add timestamp to the frame
            timestamp = time.time()
            
            # If queue is full, remove oldest frame
            if queue.full():
                try:
                    queue.get_nowait()
                except:
                    pass
            
            # Add frame to queue
            try:
                queue.put_nowait((timestamp, frame))
            except:
                pass
            
            # Sleep to maintain frame rate
            time.sleep(1.0 / self.config.CAMERA_FPS)
    
    def get_latest_frame(self, camera_id=None) -> Optional[Dict]:
        """
        Get the latest frame from the camera
        
        Args:
            camera_id: ID of the camera to get the frame from (default: None, uses config.CAMERA_INDEX)
            
        Returns:
            Dictionary with timestamp and frame, or None if no frame is available
        """
        camera_id = camera_id if camera_id is not None else self.config.CAMERA_INDEX
        
        if camera_id not in self.frame_queues:
            return None
        
        queue = self.frame_queues[camera_id]
        
        if queue.empty():
            return None
        
        # Get the latest frame
        timestamp, frame = queue.get()
        
        return {
            "timestamp": timestamp,
            "frame": frame
        }
    
    def get_frame_at_time(self, target_time, tolerance=0.05, camera_id=None) -> Optional[Dict]:
        """
        Get a frame captured at or near the specified time
        
        Args:
            target_time: Target timestamp to find a frame for
            tolerance: Time tolerance in seconds (default: 0.05)
            camera_id: ID of the camera to get the frame from (default: None, uses config.CAMERA_INDEX)
            
        Returns:
            Dictionary with timestamp and frame, or None if no matching frame is found
        """
        camera_id = camera_id if camera_id is not None else self.config.CAMERA_INDEX
        
        if camera_id not in self.frame_queues:
            return None
        
        queue = self.frame_queues[camera_id]
        
        if queue.empty():
            return None
        
        # Copy all frames to a temporary list
        frames = []
        while not queue.empty():
            try:
                frames.append(queue.get_nowait())
            except:
                break
        
        # Find the frame closest to the target time
        closest_frame = None
        min_diff = float('inf')
        
        for timestamp, frame in frames:
            diff = abs(timestamp - target_time)
            if diff < min_diff:
                min_diff = diff
                closest_frame = (timestamp, frame)
        
        # Put all frames back in the queue
        for frame in frames:
            try:
                queue.put_nowait(frame)
            except:
                break
        
        # Return the closest frame if it's within tolerance
        if closest_frame and min_diff <= tolerance:
            timestamp, frame = closest_frame
            return {
                "timestamp": timestamp,
                "frame": frame
            }
        
        return None
    
    def save_frame(self, frame, filename):
        """
        Save a frame to a file
        
        Args:
            frame: Frame to save
            filename: Filename to save to
            
        Returns:
            True if successful, False otherwise
        """
        try:
            cv2.imwrite(filename, frame)
            return True
        except Exception as e:
            self.logger.error(f"Error saving frame to {filename}: {str(e)}")
            return False
    
    def stop_capture(self, camera_id=None):
        """
        Stop capturing frames from the camera
        
        Args:
            camera_id: ID of the camera to stop capturing from (default: None, uses config.CAMERA_INDEX)
        """
        camera_id = camera_id if camera_id is not None else self.config.CAMERA_INDEX
        
        if camera_id not in self.cameras:
            return
        
        camera_info = self.cameras[camera_id]
        camera_info["running"] = False
        
        if camera_info["thread"] and camera_info["thread"].is_alive():
            camera_info["thread"].join(timeout=1.0)
        
        self.logger.info(f"Stopped capture on camera {camera_id}")
    
    def close_all(self):
        """
        Close all cameras
        """
        for camera_id in list(self.cameras.keys()):
            self.stop_capture(camera_id)
            
            camera_info = self.cameras[camera_id]
            if "capture" in camera_info:
                camera_info["capture"].release()
            
            del self.cameras[camera_id]
            
        self.logger.info("Closed all cameras")
