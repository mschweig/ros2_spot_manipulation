"""Centralized Spot SDK client management with proper resource handling."""

import logging
import time
from contextlib import contextmanager
from typing import Optional, Tuple

import bosdyn.client
import bosdyn.client.lease
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient


class SpotClientError(Exception):
    """Custom exception for Spot client operations."""
    pass


class SpotClient:
    """Centralized Spot SDK client with proper resource management."""
    
    def __init__(self, username: str, password: str, hostname: str, logger: Optional[logging.Logger] = None):
        """Initialize Spot client with credentials.
        
        Args:
            username: Spot robot username
            password: Spot robot password  
            hostname: Spot robot IP address
            logger: Optional logger instance
        """
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger or logging.getLogger(__name__)
        
        self._sdk = None
        self._robot = None
        self._lease_client = None
        self._lease_keep_alive = None
        
        # Service clients
        self._image_client = None
        self._manipulation_client = None
        self._command_client = None
        self._state_client = None
    
    def _create_sdk_and_robot(self) -> None:
        """Create SDK and robot instances."""
        try:
            self._sdk = bosdyn.client.create_standard_sdk('SpotManipulationInterface')
            self._robot = self._sdk.create_robot(self._hostname)
            self._robot.authenticate(self._username, self._password)
            self._robot.time_sync.wait_for_sync()
        except Exception as e:
            raise SpotClientError(f"Failed to create robot connection: {e}")
    
    def _ensure_robot_ready(self) -> None:
        """Ensure robot is powered on and ready."""
        if not self._robot.is_powered_on():
            self._logger.info("Powering on robot...")
            self._robot.power_on(timeout_sec=20)
            if not self._robot.is_powered_on():
                raise SpotClientError("Failed to power on robot")
    
    def _acquire_lease(self) -> None:
        """Acquire robot lease."""
        self._lease_client = self._robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name
        )
        self._lease_client.take()
        self._lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(
            self._lease_client, must_acquire=True, return_at_exit=False
        )
        self._lease_keep_alive.__enter__()
    
    def _initialize_service_clients(self) -> None:
        """Initialize all required service clients."""
        self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
        self._manipulation_client = self._robot.ensure_client(ManipulationApiClient.default_service_name)
        self._command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
    
    @contextmanager
    def get_robot_context(self):
        """Context manager for robot operations with proper cleanup."""
        try:
            self._create_sdk_and_robot()
            self._acquire_lease()
            self._ensure_robot_ready()
            self._initialize_service_clients()
            
            yield self
            
        except Exception as e:
            self._logger.error(f"Robot operation failed: {e}")
            raise SpotClientError(f"Robot operation failed: {e}")
        finally:
            self._cleanup()
    
    def _cleanup(self) -> None:
        """Clean up resources."""
        if self._lease_keep_alive:
            try:
                self._lease_keep_alive.__exit__(None, None, None)
            except Exception as e:
                self._logger.warning(f"Error during lease cleanup: {e}")
        
        # Reset clients
        self._image_client = None
        self._manipulation_client = None
        self._command_client = None
        self._state_client = None
    
    @property
    def robot(self):
        """Get robot instance."""
        if not self._robot:
            raise SpotClientError("Robot not initialized. Use get_robot_context().")
        return self._robot
    
    @property
    def image_client(self):
        """Get image client."""
        if not self._image_client:
            raise SpotClientError("Image client not initialized.")
        return self._image_client
    
    @property
    def manipulation_client(self):
        """Get manipulation client."""
        if not self._manipulation_client:
            raise SpotClientError("Manipulation client not initialized.")
        return self._manipulation_client
    
    @property
    def command_client(self):
        """Get command client."""
        if not self._command_client:
            raise SpotClientError("Command client not initialized.")
        return self._command_client
    
    @property
    def state_client(self):
        """Get state client."""
        if not self._state_client:
            raise SpotClientError("State client not initialized.")
        return self._state_client

